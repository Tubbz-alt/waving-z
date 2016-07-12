# Waving-Z

This program is able to encode and decode ITU G.9959 frames
(implemented, e.g. by the z-wave home automation protocol) using a
RTL-SDR dongle or an HackRF One (or any other radio handling raw IQ
files).

### Example command line for the EU frequency 

    rtl_sdr -f 868440000 -s 2048000 -g 25  - |./rtl_zwave


## Modulator details

The modulator is a simple FSK modulator. The modulator is phase
continuous using a simple trick (the sampling rate frequencies are multiples of the
sampling rate, makeing the modulator magically phase continuous)

"FSK measurements often require spans which are much larger than the
symbol rate because of large deviations inherent in many FSK
signals. The wide span may result in a noisy demodulated signal. The
low pass filter is a type of narrow, Gaussian filter (Gaussian shape
in the frequency domain) with a cutoff frequency greater than the
symbol rate. The filter has a 3 dB point of approximately 2/3 of the
symbol rate and is down by 40 dB at about 2x the symbol rate. Applying
the low pass filter to the measured signal may smooth the result."

http://rfmw.em.keysight.com/wireless/helpfiles/89600b/webhelp/subsystems/digdemod/content/dlg_digdemod_fltr_lowpass.htm
hackrf_transfer -f 868468000 -s 2000000 -x 20 -R -t mod-new
