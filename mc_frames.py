import sys, json, base64, logging, math
sys.path[:] = [p for p in sys.path if p not in ('/tmp','/private/tmp','')]
sys.path.insert(0, '/tmp/of-ttv/src')
from collections import deque
from datetime import datetime, timezone
import numpy as np
logging.disable(logging.WARNING)
from openflight.kld7.tracker import KLD7Tracker
from openflight.kld7.types import KLD7Frame
import openflight.kld7.two_ray as TR
SESSION='/Users/john.pacino/openflight_sessions/session_20260615_120512_range.CORRECTED.jsonl'
TARGETS={'12:17:28','12:21:09'}
ACQ=TR.ACQ_MS

sd={}; vbuf={}; rawbs={}
def sod(dt): return dt.hour*3600+dt.minute*60+dt.second+dt.microsecond/1e6
for l in open(SESSION):
    o=json.loads(l); t=o.get('type'); sn=o.get('shot_number')
    if t=='shot_detected': sd[sn]=dict(ts=o['ts'][11:19],club=o.get('club'),imp=o.get('impact_timestamp'),impk=o.get('impact_timestamp_kld7'),lv=o.get('launch_angle_vertical'))
    elif t=='rolling_buffer_capture': rawbs[sn]=o.get('ball_speed_mph')
    elif t=='kld7_buffer' and o.get('orientation')=='vertical': vbuf[sn]=o

def mkf(b): return [KLD7Frame(timestamp=d['timestamp'],radc=(base64.b64decode(d['radc_b64']) if d.get('radc_b64') else None),
    arrival_timestamp=d.get('arrival_timestamp'),complete_timestamp=d.get('complete_timestamp'),read_duration_ms=d.get('read_duration_ms'),done_frame_number=d.get('done_frame_number')) for d in b['frames']]

# wrap estimate_two_ray to capture (frames, impact_ts, diag)
CAP={}
_orig=TR.estimate_two_ray
def wrap(frames, impact_timestamp, *a, **k):
    r=_orig(frames, impact_timestamp, *a, **k)
    CAP['frames']=frames; CAP['impact']=impact_timestamp; CAP['diag']=r.diagnostics; CAP['la']=r.launch_angle_deg; CAP['conf']=r.confidence; CAP['refuse']=r.refusal_reason
    return r
TR.estimate_two_ray=wrap

def hhmmss(epoch):
    dt=datetime.fromtimestamp(epoch, tz=timezone.utc).astimezone()
    return dt.strftime('%H:%M:%S.')+ '%03d'%(dt.microsecond//1000)

for sn,s in sd.items():
    if s['ts'] not in TARGETS or sn not in vbuf or sn not in rawbs: continue
    bs=rawbs[sn]
    tr=KLD7Tracker(port=None,orientation='vertical',buffer_seconds=6.0,angle_offset_deg=1.5,range_m=5,speed_kmh=100,
        vertical_estimator='two_ray',mount_tilt_deg=10.5,ball_distance_ft=5.0,ball_above_radar_ft=-4.0/12.0,vertical_flight_window_net_distance_ft=10.0)
    tr._ring_buffer=deque(mkf(b:=vbuf[sn]),maxlen=tr.max_buffer_frames)
    CAP.clear()
    a=tr.get_angle_for_shot(shot_timestamp=s['imp'],ball_speed_mph=bs,impact_timestamp=s['impk'])
    diag=CAP.get('diag',{})
    tau=diag.get('tau_range_ms')
    impact=CAP.get('impact')
    print('='*72)
    print('shot %s   live_logged LA %s   raw_OPS_bs %.1f mph'%(s['ts'], s['lv'], bs))
    print('  impact_ts (OPS)   = %s'%hhmmss(s['imp']))
    print('  impact_ts (KLD7)  = %s   <- two_ray anchor'%hhmmss(s['impk']))
    print('  two_ray result    = LA %s  conf %s  refuse %s'%(CAP.get('la'),CAP.get('conf'),CAP.get('refuse')))
    print('  tau_range_ms      = %s   (clock offset added to every frame)'%tau)
    fr=diag.get('frames') or []
    if not fr:
        print('  no valid frames in diag (n_valid=%s)'%diag.get('n_frames_valid')); continue
    print('  valid frames used (t_ms = frame time AFTER impact, tau-corrected):')
    print('    %-13s %8s  %7s %9s %6s %7s %8s'%('abs_time','t_ms','el_deg','el_image','rho','resid','range_ft'))
    rows=sorted(fr,key=lambda d:d['t_ms'])
    for i,d in enumerate(rows):
        # back out absolute time:  t_ms = (t_raw - ACQ/2) + tau  ->  t_raw = t_ms - tau + ACQ/2
        t_raw=d['t_ms']-(tau or 0)+ACQ/2.0
        abs_ts=impact + t_raw/1000.0
        mark=' <-- EARLIEST' if i==0 else ''
        print('    %-13s %+8.1f  %7.2f %9s %6.2f %7.4f %8s%s'%(
            hhmmss(abs_ts), d['t_ms'], d['el_deg'],
            ('%.2f'%d['el_image_deg'] if d['el_image_deg'] is not None else 'nan'),
            d['rho'], d['resid'],
            ('%.2f'%d['range_ft'] if d['range_ft'] is not None else 'nan'), mark))
TR.estimate_two_ray=_orig
