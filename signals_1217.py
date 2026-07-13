import sys, json, base64, logging, math
sys.path[:] = [p for p in sys.path if p not in ('/tmp','/private/tmp','')]
sys.path.insert(0, '/tmp/of-ttv/src')
from collections import deque
from datetime import datetime
import numpy as np
logging.disable(logging.WARNING)
from openflight.kld7.tracker import KLD7Tracker
from openflight.kld7.types import KLD7Frame
import openflight.kld7.two_ray as TR
SESSION='/Users/john.pacino/openflight_sessions/session_20260615_120512_range.CORRECTED.jsonl'
TMJSON='/Users/john.pacino/openflight_sessions/trackman_06151200_7i_PW_DR.json'
def sod(dt): return dt.hour*3600+dt.minute*60+dt.second+dt.microsecond/1e6
sd={}; vbuf={}; rawbs={}
for l in open(SESSION):
    o=json.loads(l); t=o.get('type'); sn=o.get('shot_number')
    if t=='shot_detected': sd[sn]=dict(ts=o['ts'][11:19],sod=sod(datetime.fromisoformat(o['ts'])),club=o.get('club'),imp=o.get('impact_timestamp'),impk=o.get('impact_timestamp_kld7'))
    elif t=='rolling_buffer_capture': rawbs[sn]=o.get('ball_speed_mph')
    elif t=='kld7_buffer' and o.get('orientation')=='vertical': vbuf[sn]=o
d=json.load(open(TMJSON)); tm=[]
for sg in d['StrokeGroups']:
    cl={'7Iron':'7-iron','PitchingWedge':'pw','Driver':'driver'}.get(sg.get('Club'))
    for s in sg['Strokes']:
        m=s.get('Measurement') or {}; tm.append(dict(sod=sod(datetime.fromisoformat(s['Time'])),club=cl,la=float(m['LaunchAngle'])))
diffs=np.array([o['sod']-x['sod'] for o in sd.values() for x in tm]); bn=np.arange(diffs.min(),diffs.max()+2,2.0)
hh,e=np.histogram(diffs,bins=bn); pk=e[hh.argmax()]; off=float(np.median(diffs[(diffs>=pk-6)&(diffs<=pk+6)]))
tmpw=[x for x in tm if x['club']=='pw']
def mkf(b): return [KLD7Frame(timestamp=q['timestamp'],radc=(base64.b64decode(q['radc_b64']) if q.get('radc_b64') else None),
    arrival_timestamp=q.get('arrival_timestamp'),complete_timestamp=q.get('complete_timestamp'),read_duration_ms=q.get('read_duration_ms'),done_frame_number=q.get('done_frame_number')) for q in b['frames']]
work={}
for sn,s in sd.items():
    if s['club']!='pw' or sn not in vbuf or sn not in rawbs: continue
    dd,best=min(((abs((s['sod']-x['sod'])-off),x) for x in tmpw),key=lambda c:c[0])
    if dd>12: continue
    work[s['ts']]=(s,rawbs[sn],vbuf[sn],best['la'])
CAP={}; _orig=TR.estimate_two_ray
def wrap(frames, impact_timestamp, *a, **k):
    r=_orig(frames, impact_timestamp, *a, **k); CAP['d']=r.diagnostics; CAP['la']=r.launch_angle_deg; CAP['conf']=r.confidence; CAP['ref']=r.refusal_reason; return r
TR.estimate_two_ray=wrap
def run(b,bs,imp,impk):
    tr=KLD7Tracker(port=None,orientation='vertical',buffer_seconds=6.0,angle_offset_deg=1.5,range_m=5,speed_kmh=100,
        vertical_estimator='two_ray',mount_tilt_deg=10.5,ball_distance_ft=5.0,ball_above_radar_ft=-4.0/12.0,vertical_flight_window_net_distance_ft=10.0)
    tr._ring_buffer=deque(mkf(b),maxlen=tr.max_buffer_frames)
    CAP.clear(); ret=tr.get_angle_for_shot(shot_timestamp=imp,ball_speed_mph=bs,impact_timestamp=impk)
    return CAP.get('la'),CAP.get('conf'),CAP.get('d',{})
focus=['12:17:28','12:19:11','12:25:02','12:19:40','12:20:08','12:23:48']  # bad / recoverable-single / suppressed-2frame / 3 good
print('Internal (TM-free) signals — does anything flag 12:17:28?\n')
print('  ts        raw_LA conf nval | per-frame  el / el_img / sep / rho / resid')
for ts in focus:
    if ts not in work: continue
    s,bs,b,tmla=work[ts]; la,conf,dg=run(b,bs,s['imp'],s['impk'])
    cells=[]
    for f in dg.get('frames',[]):
        ei=f['el_image_deg']
        cells.append('%.1f/%s/%s/%.2f/%.3f'%(f['el_deg'],('%.1f'%ei if ei is not None else 'nan'),
            ('%.1f'%abs(f['el_deg']-ei) if ei is not None else 'nan'),f['rho'],f['resid']))
    flag='   <== the bad one' if ts=='12:17:28' else ''
    print('  %s  %5.1f  %.2f  %d   | %s%s'%(ts,la if la else 0,conf or 0,dg.get('n_frames_valid') or 0,'  ||  '.join(cells),flag))
TR.estimate_two_ray=_orig
# club band from recs.json (trusted PW = Tier-1 readings)
recs=json.load(open('/tmp/of-ttv/recs.json'))
pw_t1=[r['pos'] for r in recs if r['club']=='pw' and r['nval']>=2 and r['maxsep']>=9 and r['maxel']>=14 and r['pos'] is not None]
print('\nPW "band" from the 5 trusted Tier-1 readings: %.1f - %.1f  (mean %.1f, sd %.1f)'%(
    min(pw_t1),max(pw_t1),np.mean(pw_t1),np.std(pw_t1)))
print('  12:17:28 raw reading 16.2  ->  %.1f deg below band floor'%(min(pw_t1)-16.2))
print('  12:17:28 corrected (+4.5)  ->  20.7  ->  %.1f below band floor'%(min(pw_t1)-20.7))
