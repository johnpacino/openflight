import sys, json, base64, logging, math
sys.path[:] = [p for p in sys.path if p not in ('/tmp','/private/tmp','')]
sys.path.insert(0, '/tmp/of-ttv/src')
from collections import deque
import numpy as np
logging.disable(logging.WARNING)
from openflight.kld7.tracker import KLD7Tracker
from openflight.kld7.types import KLD7Frame
import openflight.kld7.two_ray as TR
SESSION='/Users/john.pacino/openflight_sessions/session_20260615_120512_range.CORRECTED.jsonl'
TARGET='12:21:09'
sd={}; vbuf={}; rawbs={}
for l in open(SESSION):
    o=json.loads(l); t=o.get('type'); sn=o.get('shot_number')
    if t=='shot_detected': sd[sn]=dict(ts=o['ts'][11:19],imp=o.get('impact_timestamp'),impk=o.get('impact_timestamp_kld7'),lv=o.get('launch_angle_vertical'),club=o.get('club'))
    elif t=='rolling_buffer_capture': rawbs[sn]=o.get('ball_speed_mph')
    elif t=='kld7_buffer' and o.get('orientation')=='vertical': vbuf[sn]=o
def mkf(b): return [KLD7Frame(timestamp=d['timestamp'],radc=(base64.b64decode(d['radc_b64']) if d.get('radc_b64') else None),
    arrival_timestamp=d.get('arrival_timestamp'),complete_timestamp=d.get('complete_timestamp'),read_duration_ms=d.get('read_duration_ms'),done_frame_number=d.get('done_frame_number')) for d in b['frames']]
CAP={}; _orig=TR.estimate_two_ray
def wrap(frames, impact_timestamp, *a, **k):
    r=_orig(frames, impact_timestamp, *a, **k); CAP['diag']=r.diagnostics; CAP['la']=r.launch_angle_deg; CAP['conf']=r.confidence; return r
TR.estimate_two_ray=wrap
for sn,s in sd.items():
    if s['ts']!=TARGET or sn not in vbuf or sn not in rawbs: continue
    tr=KLD7Tracker(port=None,orientation='vertical',buffer_seconds=6.0,angle_offset_deg=1.5,range_m=5,speed_kmh=100,
        vertical_estimator='two_ray',mount_tilt_deg=10.5,ball_distance_ft=5.0,ball_above_radar_ft=-4.0/12.0,vertical_flight_window_net_distance_ft=10.0)
    tr._ring_buffer=deque(mkf(vbuf[sn]),maxlen=tr.max_buffer_frames)
    tr.get_angle_for_shot(shot_timestamp=s['imp'],ball_speed_mph=rawbs[sn],impact_timestamp=s['impk'])
    d=CAP['diag']
    print('shot %s   TM 23.6   returned LA %.2f (conf %.2f)'%(s['ts'],CAP['la'],CAP['conf']))
    print('  tau_range_ms        = %s  (ONE global offset, added to every frame equally)'%d.get('tau_range_ms'))
    print('  la_curve_deg        = %s   <- TIME-based fit (uses t_center+tau); THIS is what we return'%d.get('la_curve_deg'))
    print('  la_position_deg     = %s   <- RANGE-based fit (timing-free cross-check only)'%d.get('la_position_deg'))
    print('  frames (t_ms is t_center+tau, NOT re-derived from range):')
    for fr in d.get('frames',[]):
        print('    t=%+6.1fms  el %5.2f  range %s'%(fr['t_ms'],fr['el_deg'],fr['range_ft']))
TR.estimate_two_ray=_orig
