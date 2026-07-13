import sys, json, base64, logging
sys.path[:] = [p for p in sys.path if p not in ('/tmp','/private/tmp','')]
sys.path.insert(0, '/tmp/of-ttv/src')
from collections import deque
import numpy as np
logging.disable(logging.WARNING)
from openflight.kld7.tracker import KLD7Tracker
from openflight.kld7.types import KLD7Frame

SESSION='/Users/john.pacino/openflight_sessions/session_20260615_120512_range.jsonl'
TM={'12:06:42':15.4,'12:07:11':19.5,'12:07:40':19.0,'12:08:12':21.3,'12:08:36':16.5,
'12:09:06':14.7,'12:09:34':16.7,'12:10:05':17.4,'12:10:33':9.4,'12:10:57':16.2,
'12:11:30':16.2,'12:12:09':17.0,'12:12:37':15.1,'12:13:04':17.1,'12:13:35':13.1,
'12:14:02':10.0,'12:14:22':17.1,'12:14:50':14.5,'12:15:59':20.9}
sd={}; buf={}
for l in open(SESSION):
    o=json.loads(l); t=o.get('type')
    if t=='shot_detected':
        sd[o['shot_number']]=dict(ts=o['ts'][11:19],club=o.get('club'),bs=o.get('ball_speed_mph'),
            imp=o.get('impact_timestamp'),impk=o.get('impact_timestamp_kld7'),lv=o.get('launch_angle_vertical'))
    elif t=='kld7_buffer' and o.get('orientation')=='vertical':
        buf[o['shot_number']]=o['frames']
def mkframes(raw):
    out=[]
    for d in raw:
        radc=base64.b64decode(d['radc_b64']) if d.get('radc_b64') else None
        out.append(KLD7Frame(timestamp=d['timestamp'],radc=radc,
            arrival_timestamp=d.get('arrival_timestamp'),complete_timestamp=d.get('complete_timestamp'),
            read_duration_ms=d.get('read_duration_ms'),done_frame_number=d.get('done_frame_number')))
    return out
work=[]
for sn,s in sd.items():
    if s['club']=='7-iron' and sn in buf and s['ts'] in TM:
        work.append((s, mkframes(buf[sn])))
work.sort(key=lambda w: w[0]['ts'])
def run(dist, off):
    per=[]
    for s,frames in work:
        tr=KLD7Tracker(port=None,orientation='vertical',buffer_seconds=6.0,angle_offset_deg=off,
            range_m=5,speed_kmh=100,vertical_estimator='two_ray',mount_tilt_deg=10.5,
            ball_distance_ft=dist,ball_above_radar_ft=-4.0/12.0,vertical_flight_window_net_distance_ft=10.0)
        tr._ring_buffer=deque(frames,maxlen=tr.max_buffer_frames)
        ang=tr.get_angle_for_shot(shot_timestamp=s['imp'],ball_speed_mph=s['bs'],impact_timestamp=s['impk'])
        per.append((s['ts'], ang.vertical_deg if ang else None, s['lv'], TM[s['ts']]))
    return per
def stats(per):
    e=np.abs(np.array([v-tm for ts,v,lg,tm in per if v is not None]))
    return e.mean(),np.percentile(e,50),np.percentile(e,90),len(e)
print('=== VALIDATION: harness(5.0,off1.5) vs LOGGED ===')
print('       ts harness logged    TM')
for ts,v,lg,tm in run(5.0,1.5):
    print('  %8s %s %6.1f %5.1f'%(ts,('%6.1f'%v if v is not None else '   nan'),(lg or 0),tm))
m,p50,p90,n=stats(run(5.0,1.5))
print('  -> MAE %.2f P50 %.2f P90 %.2f n=%d  (live logged: 1.92/1.22/4.52)'%(m,p50,p90,n))
print('\n=== SWEEP (7-iron) ===\n  config              MAE    P50    P90   n')
for name,dist,off in [('5.0ft off1.5 base',5.0,1.5),('5.5ft off1.5',5.5,1.5),('5.0ft off0',5.0,0.0),('5.0ft off2.5',5.0,2.5),('5.0ft off3.5',5.0,3.5)]:
    m,p50,p90,n=stats(run(dist,off)); print('  %-18s%6.2f %6.2f %6.2f %3d'%(name,m,p50,p90,n))
