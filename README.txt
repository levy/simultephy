If you want to be able to measure end-to-end delay on TCP streams over LTE, then
you need to be able to carry the exact Packet parts over, because that's where
the timestamps are.

In order to support Wifi and LTE coexistence we need to make sure that we know
the exact signal time and frequency boundaries. Signals may also interfere partially
in both time and frequency resulting in partial loss of data. How is this supposed
to be modeled?

Maybe we should rather send a separate signal for each slot towards each destination
containing only the relevant resource blocks and having a weird frequency domain
description? Should we rather send a separate signal for each receivable part (e.g RB)?