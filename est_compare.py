import sys, json, base64, logging
sys.path[:] = [p for p in sys.path if p not in ('/tmp','/private/tmp','')]
sys.path.insert(0, '/tmp/of-ttv/src')
from collections import deque
import numpy as np
logging.disable(logging.WARNING)
from openflight.kld7.tracker import KLD7Tracker
from openflight.kld7.types import KLD7Frame
F='/Users/john.pacino/openflight_sessions/session_20260615_120512_range.CORRECTED.jsonl'
TM={'12:06:42':15.4,'12:07:11':19.5,'12:07:40':19.0,'12:08:12':21.3,'12:08:36':16.5,
'12:09:06':14.7,'12:09:34':16.7,'12:10:05':17.4,'12:10:33':9.4,'12:10:57':16.2,
'12:11:30':16.2,'12:12:09':17.0,'12:12:37':15.1,'12:13:04':17.1,'12:13:35':13.1,
'12:14:02':10.0,'12:14:22':17.1,'12:14:50':14.5,'12:15:59':20.9}
BAD={'12:09:06','12:06:42','12:09:34','12:13:35','12:14:50'}
sd={}; vbuf={}; rawbs={}
for l in open(F):
    o=json.loads(l); t=o.get('type'); sn=o.get('shot_number')
    if t=='shot_detected': sd[sn]=dict(ts=o['ts'][11:19],club=o.get('club'),imp=o.get('impact_timestamp'),impk=o.get('impact_timestamp_kld7'),src=o.get('launch_angle_vertical_source'))
    elif t=='rolling_buffer_capture': rawbs[sn]=o.get('ball_speed_mph')
    elif t=='kld7_buffer' and o.get('orientation')=='vertical': vbuf[sn]=o
work=[]
for sn,s in sd.items():
    if s['club']!='7-iron' or sn not in vbuf or s['ts'] not in TM or sn not in rawbs: continue
    frames=[KLD7Frame(timestamp=d['timestamp'],radc=(base64.b64decode(d['radc_b64']) if d.get('radc_b64') else None),
        arrival_timestamp=d.get('arrival_timestamp'),complete_timestamp=d.get('complete_timestamp'),
        read_duration_ms=d.get('read_duration_ms'),done_frame_number=d.get('done_frame_number')) for d in vbuf[sn]['frames']]
    work.append((s, rawbs[sn], frames))
work.sort(key=lambda w: w[0]['ts'])
def angle(fr,bs,imp,impk,est):
    tr=KLD7Tracker(port=None,orientation='vertical',buffer_seconds=6.0,angle_offset_deg=1.5,range_m=5,speed_kmh=100,
        vertical_estimator=est,mount_tilt_deg=10.5,ball_distance_ft=5.0,ball_above_radar_ft=-4.0/12.0,vertical_flight_window_net_distance_ft=10.0)
    tr._ring_buffer=deque(fr,maxlen=tr.max_buffer_frames)
    a=tr.get_angle_for_shot(shot_timestamp=imp,ball_speed_mph=bs,impact_timestamp=impk)
    return a.vertical_deg if a else None
print('=== per-shot: two_ray vs geometry (5 bad shots first) ===')
print('  ts        TM   two_ray   geometry   (two_ray err / geom err)')
res={'two_ray':[], 'geometry':[]}
order=sorted(work, key=lambda w: (w[0]['ts'] not in BAD, w[0]['ts']))
for s,bs,fr in order:
    ts=s['ts']; tr_a=angle(fr,bs,s['imp'],s['impk'],'two_ray'); ge_a=angle(fr,bs,s['imp'],s['impk'],'geometry')
    tag='BAD ' if ts in BAD else 'good'
    te = (tr_a-TM[ts]) if tr_a is not None else None
    ge = (ge_a-TM[ts]) if ge_a is not None else None
    print('  %s %s %4.1f   %s    %s     (%s / %s)'%(tag,ts,TM[ts],
        ('%5.1f'%tr_a if tr_a is not None else ' nan '),('%5.1f'%ge_a if ge_a is not None else ' nan '),
        ('%+.1f'%te if te is not None else 'nan'),('%+.1f'%ge if ge is not None else 'nan')))
    if te is not None: res['two_ray'].append(te)
    if ge is not None: res['geometry'].append(ge)
print()
for est in ('two_ray','geometry'):
    a=np.abs(np.array(res[est])); print('  %-9s overall: MAE %.2f  P50 %.2f  P90 %.2f  n=%d'%(est,a.mean(),np.percentile(a,50),np.percentile(a,90),len(a)))
