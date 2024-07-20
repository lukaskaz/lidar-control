# lidar-control
App handling 360 distance scanner (A1M8)<br>
-----<br>
$ ./lidar-control -d /dev/ttyUSB0<br>
== [Lidar A1M8 scanner on /dev/ttyUSB1 @ 115200] menu ==<br>
1 : to get info<br>
2 : to get status<br>
3 : to get sampling time<br>
4 : to get configuration<br>
5 : to normal scanning<br>
6 : to express scanning [legacy]<br>
7 : to exit<br>
-><br>
-----<br>
**-> 1<br>**
Model: 0x18<br>
Firmware: 1.27<br>
Hardware: 0x5<br>
Serialnum: 7ded9a87c5e392d3a5e49ef0ed383d65<br>
Press enter to return to menu<br>
-----<br>
**-> 2<br>**
Current status: "Ok" [0]<br>
Press enter to return to menu<br>
-----<br>
**-> 3<br>**
Normal scan: 500ms<br>
Express scan: 250ms<br>
Press enter to return to menu<br>
-----<br>
**-> 4<br>**
Scan modes count: 4<br>
Typical scan mode: 0<br>
Info for mode 0 [Standard] -> TYPICAL<br>
        > cost per sample: 500 us<br>
        > max sample rate: 2000 sps<br>
        > max distance: 12m<br>
        > answer cmd type: 0x81<br>
<br>
Info for mode 1 [Express]<br>
        > cost per sample: 250 us<br>
        > max sample rate: 4000 sps<br>
        > max distance: 12m<br>
        > answer cmd type: 0x82<br>
<br>
Info for mode 2 [Boost]<br>
        > cost per sample: 125 us<br>
        > max sample rate: 8000 sps<br>
        > max distance: 12m<br>
        > answer cmd type: 0x84<br>
<br>
Info for mode 3 [Stability]<br>
        > cost per sample: 250 us<br>
        > max sample rate: 4000 sps<br>
        > max distance: 12m<br>
        > answer cmd type: 0x84<br>
Press enter to return to menu<br>
-----<br>
**-> 5<br>**
Normal 360 scan started @ Sat Jul 20 02:40:37 2024<br>
<br>
[01] angle(dgr) 000, dist(cm): 0078.7<br>
[02] angle(dgr) 031, dist(cm): 0095.2<br>
[03] angle(dgr) 059, dist(cm): 0045.4<br>
[04] angle(dgr) 091, dist(cm): 0032.5<br>
[05] angle(dgr) 121, dist(cm): 0083.0<br>
[06] angle(dgr) 149, dist(cm): 0433.9<br>
[07] angle(dgr) 180, dist(cm): 0344.0<br>
[08] angle(dgr) 210, dist(cm): 0175.3<br>
[09] angle(dgr) 241, dist(cm): 0080.2<br>
[10] angle(dgr) 270, dist(cm): 0033.4<br>
[11] angle(dgr) 301, dist(cm): 0036.9<br>
[12] angle(dgr) 331, dist(cm): 0087.8<br>
<br>
[180dgr@344cm] GOOD: OBSTACLE FAR AWAY<br>
<br>
[000dgr@078cm] GOOD: OBSTACLE FAR AWAY<br>
[180dgr@344cm] GOOD: OBSTACLE FAR AWAY<br>
<br>
-----<br>
**-> 6<br>**
Express 360 "legacy" scan started @ Sat Jul 20 02:41:38 2024<br>
<br>
[01] angle(dgr) 001, dist(cm): 0078.6<br>
[02] angle(dgr) 031, dist(cm): 0076.4<br>
[03] angle(dgr) 061, dist(cm): 0046.3<br>
[04] angle(dgr) 091, dist(cm): 0030.7<br>
[05] angle(dgr) 121, dist(cm): 0074.5<br>
[06] angle(dgr) 151, dist(cm): 0458.1<br>
[07] angle(dgr) 181, dist(cm): 0384.4<br>
[08] angle(dgr) 210, dist(cm): 0257.6<br>
[09] angle(dgr) 241, dist(cm): 0086.5<br>
[10] angle(dgr) 271, dist(cm): 0033.3<br>
[11] angle(dgr) 299, dist(cm): 0035.4<br>
[12] angle(dgr) 331, dist(cm): 0072.1<br>
<br>
[181dgr@384cm] GOOD: OBSTACLE FAR AWAY<br>
<br>
[001dgr@078cm] GOOD: OBSTACLE FAR AWAY<br>
[181dgr@384cm] GOOD: OBSTACLE FAR AWAY<br>
<br>
-----<br>
**-> 7<br>**
Cleaning and closing<br>
$
