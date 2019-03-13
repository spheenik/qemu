/* public domain */
#include "qemu/osdep.h"
#include "qemu-common.h"
#include "audio.h"
#include "qapi/opts-visitor.h"

#include <pulse/pulseaudio.h>

#define AUDIO_CAP "pulseaudio"
#define DEBUG
#include "audio_int.h"
#include "audio_pt_int.h"

typedef struct {
    Audiodev *dev;
    pa_threaded_mainloop *mainloop;
    pa_context *context;
} paaudio;

typedef struct {
    HWVoiceOut hw;
    pa_stream *stream;
    paaudio *g;
} PAVoiceOut;

typedef struct {
    HWVoiceIn hw;
    pa_stream *stream;
    paaudio *g;
} PAVoiceIn;

static void qpa_audio_fini(void *opaque);

static void GCC_FMT_ATTR (2, 3) qpa_logerr (int err, const char *fmt, ...)
{
    va_list ap;

    va_start (ap, fmt);
    AUD_vlog (AUDIO_CAP, fmt, ap);
    va_end (ap);

    AUD_log (AUDIO_CAP, "Reason: %s\n", pa_strerror (err));
}

#ifndef PA_CONTEXT_IS_GOOD
static inline int PA_CONTEXT_IS_GOOD(pa_context_state_t x)
{
    return
            x == PA_CONTEXT_CONNECTING ||
            x == PA_CONTEXT_AUTHORIZING ||
            x == PA_CONTEXT_SETTING_NAME ||
            x == PA_CONTEXT_READY;
}
#endif

#ifndef PA_STREAM_IS_GOOD
static inline int PA_STREAM_IS_GOOD(pa_stream_state_t x)
{
    return
            x == PA_STREAM_CREATING ||
            x == PA_STREAM_READY;
}
#endif

#define CHECK_SUCCESS_GOTO(c, rerror, expression, label)        \
    do {                                                        \
        if (!(expression)) {                                    \
            if (rerror) {                                       \
                *(rerror) = pa_context_errno ((c)->context);    \
            }                                                   \
            goto label;                                         \
        }                                                       \
    } while (0);

#define CHECK_DEAD_GOTO(c, stream, rerror, label)                       \
    do {                                                                \
        if (!(c)->context || !PA_CONTEXT_IS_GOOD (pa_context_get_state((c)->context)) || \
            !(stream) || !PA_STREAM_IS_GOOD (pa_stream_get_state ((stream)))) { \
            if (((c)->context && pa_context_get_state ((c)->context) == PA_CONTEXT_FAILED) || \
                ((stream) && pa_stream_get_state ((stream)) == PA_STREAM_FAILED)) { \
                if (rerror) {                                           \
                    *(rerror) = pa_context_errno ((c)->context);        \
                }                                                       \
            } else {                                                    \
                if (rerror) {                                           \
                    *(rerror) = PA_ERR_BADSTATE;                        \
                }                                                       \
            }                                                           \
            goto label;                                                 \
        }                                                               \
    } while (0);

static int qpa_run_out(HWVoiceOut *hw, int live)
{
    PAVoiceOut *pa = (PAVoiceOut *) hw;
    int rpos, decr, samples;
    size_t avail_bytes, max_bytes;
    struct st_sample *src;
    void *pa_dst;
    int error = 0;
    int *rerror = &error;
    int r;

    decr = 0;
    rpos = hw->rpos;

    pa_threaded_mainloop_lock(pa->g->mainloop);
    CHECK_DEAD_GOTO(pa->g, pa->stream, rerror, fail);

    avail_bytes = (size_t) live << hw->info.shift;

    max_bytes = pa_stream_writable_size(pa->stream);
    CHECK_SUCCESS_GOTO(pa->g, rerror, max_bytes != -1, fail);

    samples = (int)(audio_MIN(avail_bytes, max_bytes)) >> hw->info.shift;
    while (samples) {
        int convert_samples = audio_MIN(samples, hw->samples - rpos);
        size_t b_wanted = (size_t) convert_samples << hw->info.shift;
        size_t b_effective = b_wanted;

        r = pa_stream_begin_write(pa->stream, &pa_dst, &b_effective);
        CHECK_SUCCESS_GOTO(pa->g, rerror, r == 0, fail);
        CHECK_SUCCESS_GOTO(pa->g, (int *)0, b_effective == b_wanted, fail);

        src = hw->mix_buf + rpos;
        hw->clip(pa_dst, src, convert_samples);

        r = pa_stream_write(pa->stream, pa_dst, b_effective,
                            NULL, 0LL, PA_SEEK_RELATIVE);
        CHECK_SUCCESS_GOTO(pa->g, rerror, r >= 0, fail);

        rpos = (rpos + convert_samples) % hw->samples;
        samples -= convert_samples;
        decr += convert_samples;
    }

    bail:
    pa_threaded_mainloop_unlock(pa->g->mainloop);

    hw->rpos = rpos;
    return decr;

    fail:
    qpa_logerr(error, "qpa_run_out failed\n");
    goto bail;
}

static int qpa_write (SWVoiceOut *sw, void *buf, int len)
{
    return audio_pcm_sw_write (sw, buf, len);
}

static int qpa_run_in(HWVoiceIn *hw)
{
    PAVoiceIn *pa = (PAVoiceIn *) hw;
    int wpos, incr;
    char *pa_src;
    int error = 0;
    int *rerror = &error;
    int r;
    size_t pa_avail;
    incr = 0;
    wpos = hw->wpos;

    pa_threaded_mainloop_lock(pa->g->mainloop);
    CHECK_DEAD_GOTO(pa->g, pa->stream, rerror, fail);

    size_t bytes_wanted = ((unsigned int)
            (hw->samples - audio_pcm_hw_get_live_in(hw)) << hw->info.shift);

    if (bytes_wanted == 0) {
        /* no room */
        goto bail;
    }

    size_t bytes_avail = pa_stream_readable_size(pa->stream);

    if (bytes_wanted > bytes_avail) {
        bytes_wanted = bytes_avail;
    }

    while (bytes_wanted) {
        r = pa_stream_peek(pa->stream, (const void **)&pa_src, &pa_avail);
        CHECK_SUCCESS_GOTO(pa->g, rerror, r == 0, fail);

        if (pa_avail == 0 || pa_avail > bytes_wanted) {
            break;
        }

        bytes_wanted -= pa_avail;

        while (pa_avail) {
            int chunk = audio_MIN(
                    (int)(pa_avail >> hw->info.shift), hw->samples - wpos);
            hw->conv(hw->conv_buf + wpos, pa_src, chunk);
            wpos = (wpos + chunk) % hw->samples;
            pa_src += chunk << hw->info.shift;
            pa_avail -= chunk << hw->info.shift;
            incr += chunk;
        }

        r = pa_stream_drop(pa->stream);
        CHECK_SUCCESS_GOTO(pa->g, rerror, r == 0, fail);
    }

    bail:
    pa_threaded_mainloop_unlock(pa->g->mainloop);

    hw->wpos = wpos;
    return incr;

    fail:
    qpa_logerr(error, "qpa_run_in failed\n");
    goto bail;
}

static int qpa_read (SWVoiceIn *sw, void *buf, int len)
{
    return audio_pcm_sw_read (sw, buf, len);
}

static pa_sample_format_t audfmt_to_pa (AudioFormat afmt, int endianness)
{
    int format;

    switch (afmt) {
    case AUDIO_FORMAT_S8:
    case AUDIO_FORMAT_U8:
        format = PA_SAMPLE_U8;
        break;
    case AUDIO_FORMAT_S16:
    case AUDIO_FORMAT_U16:
        format = endianness ? PA_SAMPLE_S16BE : PA_SAMPLE_S16LE;
        break;
    case AUDIO_FORMAT_S32:
    case AUDIO_FORMAT_U32:
        format = endianness ? PA_SAMPLE_S32BE : PA_SAMPLE_S32LE;
        break;
    default:
        dolog ("Internal logic error: Bad audio format %d\n", afmt);
        format = PA_SAMPLE_U8;
        break;
    }
    return format;
}

static AudioFormat pa_to_audfmt (pa_sample_format_t fmt, int *endianness)
{
    switch (fmt) {
    case PA_SAMPLE_U8:
        return AUDIO_FORMAT_U8;
    case PA_SAMPLE_S16BE:
        *endianness = 1;
        return AUDIO_FORMAT_S16;
    case PA_SAMPLE_S16LE:
        *endianness = 0;
        return AUDIO_FORMAT_S16;
    case PA_SAMPLE_S32BE:
        *endianness = 1;
        return AUDIO_FORMAT_S32;
    case PA_SAMPLE_S32LE:
        *endianness = 0;
        return AUDIO_FORMAT_S32;
    default:
        dolog ("Internal logic error: Bad pa_sample_format %d\n", fmt);
        return AUDIO_FORMAT_U8;
    }
}

static void context_state_cb (pa_context *c, void *userdata)
{
    paaudio *g = userdata;

    switch (pa_context_get_state(c)) {
    case PA_CONTEXT_READY:
    case PA_CONTEXT_TERMINATED:
    case PA_CONTEXT_FAILED:
        pa_threaded_mainloop_signal (g->mainloop, 0);
        break;

    case PA_CONTEXT_UNCONNECTED:
    case PA_CONTEXT_CONNECTING:
    case PA_CONTEXT_AUTHORIZING:
    case PA_CONTEXT_SETTING_NAME:
        break;
    }
}

static void stream_state_cb (pa_stream *s, void * userdata)
{
    paaudio *g = userdata;

    switch (pa_stream_get_state (s)) {

    case PA_STREAM_READY:
    case PA_STREAM_FAILED:
    case PA_STREAM_TERMINATED:
        pa_threaded_mainloop_signal (g->mainloop, 0);
        break;

    case PA_STREAM_UNCONNECTED:
    case PA_STREAM_CREATING:
        break;
    }
}

static pa_stream *qpa_simple_new (
        paaudio *g,
        const char *name,
        pa_stream_direction_t dir,
        const char *dev,
        const pa_sample_spec *ss,
        const pa_channel_map *map,
        const pa_buffer_attr *attr,
        bool adjust_latency,
        int *rerror)
{
    int r;
    pa_stream *stream;

    pa_threaded_mainloop_lock (g->mainloop);

    stream = pa_stream_new (g->context, name, ss, map);
    if (!stream) {
        goto fail;
    }

    pa_stream_set_state_callback (stream, stream_state_cb, g);

    if (dir == PA_STREAM_PLAYBACK) {
        r = pa_stream_connect_playback(stream, dev, attr,
             PA_STREAM_INTERPOLATE_TIMING
             | (adjust_latency ? PA_STREAM_ADJUST_LATENCY : 0)
             | PA_STREAM_AUTO_TIMING_UPDATE, NULL, NULL);
    } else {
        r = pa_stream_connect_record(stream, dev, attr,
             PA_STREAM_INTERPOLATE_TIMING
             | (adjust_latency ? PA_STREAM_ADJUST_LATENCY : 0)
             | PA_STREAM_AUTO_TIMING_UPDATE);
    }

    if (r < 0) {
        goto fail;
    }

    pa_threaded_mainloop_unlock (g->mainloop);

    return stream;

    fail:
    pa_threaded_mainloop_unlock (g->mainloop);

    if (stream) {
        pa_stream_unref (stream);
    }

    *rerror = pa_context_errno (g->context);

    return NULL;
}

static int qpa_init_out(HWVoiceOut *hw, struct audsettings *as,
                        void *drv_opaque)
{
    int error;
    pa_sample_spec ss;
    pa_buffer_attr ba;
    struct audsettings obt_as = *as;
    PAVoiceOut *pa = (PAVoiceOut *) hw;
    paaudio *g = pa->g = drv_opaque;
    AudiodevPaOptions *popts = &g->dev->u.pa;
    AudiodevPaOutOptions *ppdo = popts->out;

    uint32_t buffer_length =
            ppdo->has_buffer_length ? ppdo->buffer_length : g->dev->timer_period * 5 / 2;
    uint32_t tlength =
            ppdo->has_tlength ? ppdo->tlength : g->dev->timer_period *5 / 2;
    bool adjust_latency =
            ppdo->has_adjust_latency ? ppdo->adjust_latency : false;

    ldebug("tick duration: %i µs\n", g->dev->timer_period);
    ldebug("OUT internal buffer: %i µs\n", buffer_length);
    ldebug("OUT tlength: %i µs\n", tlength);
    ldebug("OUT adjust latency: %s\n", adjust_latency ? "yes" : "no");

    ss.format = audfmt_to_pa(as->fmt, as->endianness);
    ss.channels = as->nchannels;
    ss.rate = as->freq;

    ba.tlength = audio_buffer_bytes(ppdo, as, tlength);
    ba.maxlength = -1;
    ba.minreq = -1;
    ba.prebuf = -1;

    obt_as.fmt = pa_to_audfmt(ss.format, &obt_as.endianness);

    pa->stream = qpa_simple_new(
            g,
            "qemu",
            PA_STREAM_PLAYBACK,
            ppdo->has_name ? ppdo->name : NULL,
            &ss,
            NULL,               /* channel map */
            &ba,                /* buffering attributes */
            adjust_latency,
            &error
    );
    if (!pa->stream) {
        qpa_logerr (error, "pa_simple_new for playback failed\n");
        goto fail1;
    }

    audio_pcm_init_info(&hw->info, &obt_as);
    hw->samples = audio_buffer_samples(
            qapi_AudiodevPaPerDirectionOptions_base(ppdo), &obt_as, buffer_length);

    return 0;

    fail1:
    return -1;
}

static int qpa_init_in(HWVoiceIn *hw, struct audsettings *as, void *drv_opaque)
{
    int error;
    pa_sample_spec ss;
    pa_buffer_attr ba;
    struct audsettings obt_as = *as;
    PAVoiceIn *pa = (PAVoiceIn *) hw;
    paaudio *g = pa->g = drv_opaque;
    AudiodevPaOptions *popts = &g->dev->u.pa;
    AudiodevPaInOptions *ppdo = popts->in;

    uint32_t buffer_length =
            ppdo->has_buffer_length ? ppdo->buffer_length : g->dev->timer_period * 5 / 2;
    uint32_t fragsize =
            ppdo->has_fragsize ? ppdo->fragsize : g->dev->timer_period;
    uint32_t maxlength =
            ppdo->has_maxlength ? ppdo->maxlength : g->dev->timer_period * 2;
    bool adjust_latency =
            ppdo->has_adjust_latency ? ppdo->adjust_latency : true;

    ldebug("IN internal buffer: %i µs\n", buffer_length);
    ldebug("IN fragsize: %i µs\n", fragsize);
    ldebug("IN maxlength: %i µs\n", maxlength);
    ldebug("IN adjust latency: %s\n", adjust_latency ? "yes" : "no");

    ss.format = audfmt_to_pa(as->fmt, as->endianness);
    ss.channels = as->nchannels;
    ss.rate = as->freq;

    ba.fragsize = audio_buffer_bytes(ppdo, as, fragsize);
    ba.maxlength = -1; //audio_buffer_bytes(ppdo, as, maxlength);
    ba.minreq = -1;
    ba.prebuf = -1;

    obt_as.fmt = pa_to_audfmt(ss.format, &obt_as.endianness);

    pa->stream = qpa_simple_new(
            g,
            "qemu",
            PA_STREAM_RECORD,
            ppdo->has_name ? ppdo->name : NULL,
            &ss,
            NULL,                   /* channel map */
            &ba,                    /* buffering attributes */
            adjust_latency,
            &error
    );
    if (!pa->stream) {
        qpa_logerr (error, "pa_simple_new for capture failed\n");
        goto fail1;
    }

    audio_pcm_init_info(&hw->info, &obt_as);
    hw->samples = audio_buffer_samples(
            qapi_AudiodevPaPerDirectionOptions_base(ppdo), &obt_as, buffer_length);

    return 0;

    fail1:
    return -1;
}

static void qpa_fini_out (HWVoiceOut *hw)
{
    PAVoiceOut *pa = (PAVoiceOut *) hw;

    if (pa->stream) {
        pa_stream_unref (pa->stream);
        pa->stream = NULL;
    }
}

static void qpa_fini_in (HWVoiceIn *hw)
{
    PAVoiceIn *pa = (PAVoiceIn *) hw;

    if (pa->stream) {
        pa_stream_unref (pa->stream);
        pa->stream = NULL;
    }
}

static int qpa_ctl_out (HWVoiceOut *hw, int cmd, ...)
{
    PAVoiceOut *pa = (PAVoiceOut *) hw;
    pa_operation *op;
    pa_cvolume v;
    paaudio *g = pa->g;

#ifdef PA_CHECK_VERSION    /* macro is present in 0.9.16+ */
    pa_cvolume_init (&v);  /* function is present in 0.9.13+ */
#endif

    switch (cmd) {
    case VOICE_VOLUME:
        {
            SWVoiceOut *sw;
            va_list ap;

            va_start (ap, cmd);
            sw = va_arg (ap, SWVoiceOut *);
            va_end (ap);

            v.channels = 2;
            v.values[0] = ((PA_VOLUME_NORM - PA_VOLUME_MUTED) * sw->vol.l) / UINT32_MAX;
            v.values[1] = ((PA_VOLUME_NORM - PA_VOLUME_MUTED) * sw->vol.r) / UINT32_MAX;

            pa_threaded_mainloop_lock (g->mainloop);

            op = pa_context_set_sink_input_volume (g->context,
                pa_stream_get_index (pa->stream),
                &v, NULL, NULL);
            if (!op)
                qpa_logerr (pa_context_errno (g->context),
                            "set_sink_input_volume() failed\n");
            else
                pa_operation_unref (op);

            op = pa_context_set_sink_input_mute (g->context,
                pa_stream_get_index (pa->stream),
                sw->vol.mute, NULL, NULL);
            if (!op) {
                qpa_logerr (pa_context_errno (g->context),
                            "set_sink_input_mute() failed\n");
            } else {
                pa_operation_unref (op);
            }

            pa_threaded_mainloop_unlock (g->mainloop);
        }
    }
    return 0;
}

static int qpa_ctl_in (HWVoiceIn *hw, int cmd, ...)
{
    PAVoiceIn *pa = (PAVoiceIn *) hw;
    pa_operation *op;
    pa_cvolume v;
    paaudio *g = pa->g;

#ifdef PA_CHECK_VERSION
    pa_cvolume_init (&v);
#endif

    switch (cmd) {
    case VOICE_VOLUME:
        {
            SWVoiceIn *sw;
            va_list ap;

            va_start (ap, cmd);
            sw = va_arg (ap, SWVoiceIn *);
            va_end (ap);

            v.channels = 2;
            v.values[0] = ((PA_VOLUME_NORM - PA_VOLUME_MUTED) * sw->vol.l) / UINT32_MAX;
            v.values[1] = ((PA_VOLUME_NORM - PA_VOLUME_MUTED) * sw->vol.r) / UINT32_MAX;

            pa_threaded_mainloop_lock (g->mainloop);

            op = pa_context_set_source_output_volume (g->context,
                pa_stream_get_index (pa->stream),
                &v, NULL, NULL);
            if (!op) {
                qpa_logerr (pa_context_errno (g->context),
                            "set_source_output_volume() failed\n");
            } else {
                pa_operation_unref(op);
            }

            op = pa_context_set_source_output_mute (g->context,
                pa_stream_get_index (pa->stream),
                sw->vol.mute, NULL, NULL);
            if (!op) {
                qpa_logerr (pa_context_errno (g->context),
                            "set_source_output_mute() failed\n");
            } else {
                pa_operation_unref (op);
            }

            pa_threaded_mainloop_unlock (g->mainloop);
        }
    }
    return 0;
}

/* common */
static void *qpa_audio_init(Audiodev *dev)
{
    paaudio *g;
    AudiodevPaOptions *popts = &dev->u.pa;
    const char *server;

    if (!popts->has_server) {
        char pidfile[64];
        char *runtime;
        struct stat st;

        runtime = getenv("XDG_RUNTIME_DIR");
        if (!runtime) {
            return NULL;
        }
        snprintf(pidfile, sizeof(pidfile), "%s/pulse/pid", runtime);
        if (stat(pidfile, &st) != 0) {
            return NULL;
        }
    }

    assert(dev->driver == AUDIODEV_DRIVER_PA);

    g = g_malloc(sizeof(paaudio));
    server = popts->has_server ? popts->server : NULL;

    g->dev = dev;
    g->mainloop = NULL;
    g->context = NULL;

    g->mainloop = pa_threaded_mainloop_new ();
    if (!g->mainloop) {
        goto fail;
    }

    g->context = pa_context_new (pa_threaded_mainloop_get_api (g->mainloop),
                                 server);
    if (!g->context) {
        goto fail;
    }

    pa_context_set_state_callback (g->context, context_state_cb, g);

    if (pa_context_connect (g->context, server, 0, NULL) < 0) {
        qpa_logerr (pa_context_errno (g->context),
                    "pa_context_connect() failed\n");
        goto fail;
    }

    pa_threaded_mainloop_lock (g->mainloop);

    if (pa_threaded_mainloop_start (g->mainloop) < 0) {
        goto unlock_and_fail;
    }

    for (;;) {
        pa_context_state_t state;

        state = pa_context_get_state (g->context);

        if (state == PA_CONTEXT_READY) {
            break;
        }

        if (!PA_CONTEXT_IS_GOOD (state)) {
            qpa_logerr (pa_context_errno (g->context),
                        "Wrong context state\n");
            goto unlock_and_fail;
        }

        /* Wait until the context is ready */
        pa_threaded_mainloop_wait (g->mainloop);
    }

    pa_threaded_mainloop_unlock (g->mainloop);

    return g;

    unlock_and_fail:
    pa_threaded_mainloop_unlock (g->mainloop);
    fail:
    AUD_log (AUDIO_CAP, "Failed to initialize PA context");
    qpa_audio_fini(g);
    return NULL;
}

static void qpa_audio_fini (void *opaque)
{
    paaudio *g = opaque;

    if (g->mainloop) {
        pa_threaded_mainloop_stop (g->mainloop);
    }

    if (g->context) {
        pa_context_disconnect (g->context);
        pa_context_unref (g->context);
    }

    if (g->mainloop) {
        pa_threaded_mainloop_free (g->mainloop);
    }

    g_free(g);
}

static struct audio_pcm_ops qpa_pcm_ops = {
    .init_out = qpa_init_out,
    .fini_out = qpa_fini_out,
    .run_out  = qpa_run_out,
    .write    = qpa_write,
    .ctl_out  = qpa_ctl_out,

    .init_in  = qpa_init_in,
    .fini_in  = qpa_fini_in,
    .run_in   = qpa_run_in,
    .read     = qpa_read,
    .ctl_in   = qpa_ctl_in
};

static struct audio_driver pa_audio_driver = {
    .name           = "pa",
    .descr          = "http://www.pulseaudio.org/",
    .init           = qpa_audio_init,
    .fini           = qpa_audio_fini,
    .pcm_ops        = &qpa_pcm_ops,
    .can_be_default = 1,
    .max_voices_out = INT_MAX,
    .max_voices_in  = INT_MAX,
    .voice_size_out = sizeof (PAVoiceOut),
    .voice_size_in  = sizeof (PAVoiceIn),
    .ctl_caps       = VOICE_VOLUME_CAP
};

static void register_audio_pa(void)
{
    audio_driver_register(&pa_audio_driver);
}
type_init(register_audio_pa);
