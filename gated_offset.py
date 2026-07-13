import sys, json, base64, logging
sys.path[:] = [p for p in sys.path if p not in ('/tmp','/private/tmp','')]
sys.path.insert(0, '/tmp/of-ttv/src')
from collections import deque
import numpy as np
logging.disable(logging.WARNING)
from openflight.kld7.tracker import KLD7Tracker
from openflight.kld7.types import KLD7Frame
from openflight.kld7 import two_ray as TR
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
def frames_obj(sn):
    return [KLD7Frame(timestamp=d['timestamp'],radc=(base64.b64decode(d['radc_b64']) if d.get('radc_b64') else None),
        arrival_timestamp=d.get('arrival_timestamp'),complete_timestamp=d.get('complete_timestamp'),
        read_duration_ms=d.get('read_duration_ms'),done_frame_number=d.get('done_frame_number')) for d in vbuf[sn]['frames']]
def frames_dict(sn):
    out=[]
    for d in vbuf[sn]['frames']:
        fd={'timestamp':d['timestamp']}
        if d.get('radc_b64'): fd['radc']=base64.b64decode(d['radc_b64'])
        out.append(fd)
    return out
shots=[]  # (ts, faithful_angle, minsep)
for sn,s in sd.items():
    if s['club']!='7-iron' or sn not in vbuf or s['ts'] not in TM or sn not in rawbs or s['src']!='radar': continue
    tr=KLD7Tracker(port=None,orientation='vertical',buffer_seconds=6.0,angle_offset_deg=1.5,range_m=5,speed_kmh=100,
        vertical_estimator='two_ray',mount_tilt_deg=10.5,ball_distance_ft=5.0,ball_above_radar_ft=-4.0/12.0,vertical_flight_window_net_distance_ft=10.0)
    tr._ring_buffer=deque(frames_obj(sn),maxlen=tr.max_buffer_frames)
    a=tr.get_angle_for_shot(shot_timestamp=s['imp'],ball_speed_mph=rawbs[sn],impact_timestamp=s['impk'])
    ang=a.vertical_deg if a else None
    res=TR.estimate_two_ray(frames_dict(sn), impact_timestamp=s['impk'], ball_speed_mph=rawbs[sn],
        mount_deg=10.5, angle_offset_deg=1.5, distance_ft=5.0, ball_above_radar_ft=-4.0/12.0, net_distance_ft=10.0, range_m=5.0)
    fr=res.diagnostics.get('frames') or []
    seps=[abs(f['el_deg']-f['el_image_deg']) for f in fr if f.get('el_image_deg') is not None]
    minsep=min(seps) if seps else None
    shots.append((s['ts'], ang, minsep))
def mae(es): a=np.abs(np.array(es)); return a.mean(),np.percentile(a,50),np.percentile(a,90)
THR=2.5
print('per-shot gated-correction effect (THR sep<%.1f, +4 deg):'%THR)
print('  ts        TM   ang  minsep  merged?  +4corr  err_base  err_corr')
for ts,ang,ms in sorted(shots, key=lambda x:(x[0] not in BAD, x[0])):
    if ang is None: continue
    merged = (ms is not None and ms<THR)
    corr = ang+4.0 if merged else ang
    print('  %s%s %4.1f %5.1f %6s   %-5s   %5.1f   %+5.1f   %+5.1f'%(
        ('*' if ts in BAD else ' '),ts,TM[ts],ang,('%.1f'%ms if ms is not None else 'na'),merged,corr,ang-TM[ts],corr-TM[ts]))
print()
for corr in [0,3,4,5]:
    es=[]
    for ts,ang,ms in shots:
        if ang is None: continue
        a = ang + (corr if (ms is not None and ms<THR) else 0)
        es.append(a-TM[ts])
    m,p50,p90=mae(es); print('  +%d deg gated (sep<%.1f): MAE %.2f  P50 %.2f  P90 %.2f  (n=%d)'%(corr,THR,m,p50,p90,len(es)))
