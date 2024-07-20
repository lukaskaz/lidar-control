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
[02] angle(dgr) 031, dist(cm): 0090.3<br>
[03] angle(dgr) 059, dist(cm): 0127.1<br>
[04] angle(dgr) 091, dist(cm): 0030.8<br>
[05] angle(dgr) 121, dist(cm): 0083.2<br>
[06] angle(dgr) 150, dist(cm): 0433.2<br>
[07] angle(dgr) 181, dist(cm): 0343.2<br>
[08] angle(dgr) 210, dist(cm): 0174.8<br>
[09] angle(dgr) 241, dist(cm): 0080.3<br>
[10] angle(dgr) 270, dist(cm): 0033.4<br>
[11] angle(dgr) 300, dist(cm): 0036.8<br>
[12] angle(dgr) 331, dist(cm): 0087.9<br>
<br>
[181dgr@343cm] GOOD: OBSTACLE FAR AWAY<br>
<br>
[000dgr@078cm] GOOD: OBSTACLE FAR AWAY<br>
[181dgr@343cm] GOOD: OBSTACLE FAR AWAY<br>
<br>
-----<br>
**-> 6<br>**
Express 360 "legacy" scan started @ Sat Jul 20 02:41:38 2024<br>
<br>
[01] angle(dgr) 001, dist(cm): 0078.7<br>
[02] angle(dgr) 031, dist(cm): 0095.8<br>
[03] angle(dgr) 059, dist(cm): 0127.4<br>
[04] angle(dgr) 090, dist(cm): 0031.5<br>
[05] angle(dgr) 121, dist(cm): 0083.0<br>
[06] angle(dgr) 150, dist(cm): 0432.2<br>
[07] angle(dgr) 181, dist(cm): 0343.2<br>
[08] angle(dgr) 211, dist(cm): 0174.7<br>
[09] angle(dgr) 240, dist(cm): 0081.3<br>
[10] angle(dgr) 270, dist(cm): 0033.4<br>
[11] angle(dgr) 301, dist(cm): 0037.0<br>
[12] angle(dgr) 331, dist(cm): 0087.9<br>
<br>
[181dgr@343cm] GOOD: OBSTACLE FAR AWAY<br>
<br>
[001dgr@078cm] GOOD: OBSTACLE FAR AWAY<br>
[181dgr@343cm] GOOD: OBSTACLE FAR AWAY<br>
<br>
-----<br>
**-> 7<br>**
Cleaning and closing<br>
$
