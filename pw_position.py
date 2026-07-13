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
    if t=='shot_detected': sd[sn]=dict(ts=o['ts'][11:19],sod=sod(datetime.fromisoformat(o['ts'])),club=o.get('club'),imp=o.get('impact_timestamp'),impk=o.get('impact_timestamp_kld7'),lv=o.get('launch_angle_vertical'))
    elif t=='rolling_buffer_capture': rawbs[sn]=o.get('ball_speed_mph')
    elif t=='kld7_buffer' and o.get('orientation')=='vertical': vbuf[sn]=o
d=json.load(open(TMJSON)); tm=[]
for sg in d['StrokeGroups']:
    cl={'7Iron':'7-iron','PitchingWedge':'pw','Driver':'driver'}.get(sg.get('Club'))
    for s in sg['Strokes']:
        m=s.get('Measurement') or {}; tm.append(dict(sod=sod(datetime.fromisoformat(s['Time'])),club=cl,la=float(m['LaunchAngle'])))
diffs=np.array([o['sod']-x['sod'] for o in sd.values() for x in tm]); bn=np.arange(diffs.min(),diffs.max()+2,2.0)
h,e=np.histogram(diffs,bins=bn); pk=e[h.argmax()]; off=float(np.median(diffs[(diffs>=pk-6)&(diffs<=pk+6)]))
tmpw=[x for x in tm if x['club']=='pw']
def mkf(b): return [KLD7Frame(timestamp=d['timestamp'],radc=(base64.b64decode(d['radc_b64']) if d.get('radc_b64') else None),
    arrival_timestamp=d.get('arrival_timestamp'),complete_timestamp=d.get('complete_timestamp'),read_duration_ms=d.get('read_duration_ms'),done_frame_number=d.get('done_frame_number')) for d in b['frames']]
work=[]
for sn,s in sd.items():
    if s['club']!='pw' or sn not in vbuf or sn not in rawbs: continue
    dd,best=min(((abs((s['sod']-x['sod'])-off),x) for x in tmpw),key=lambda c:c[0])
    if dd>12: continue
    work.append((s,rawbs[sn],vbuf[sn],best['la']))
work.sort(key=lambda w:w[0]['ts'])
CAP={}; _orig=TR.estimate_two_ray
def wrap(frames, impact_timestamp, *a, **k):
    r=_orig(frames, impact_timestamp, *a, **k); CAP['diag']=r.diagnostics; return r
TR.estimate_two_ray=wrap
def runshot(b,bs,imp,impk):
    tr=KLD7Tracker(port=None,orientation='vertical',buffer_seconds=6.0,angle_offset_deg=1.5,range_m=5,speed_kmh=100,
        vertical_estimator='two_ray',mount_tilt_deg=10.5,ball_distance_ft=5.0,ball_above_radar_ft=-4.0/12.0,vertical_flight_window_net_distance_ft=10.0)
    tr._ring_buffer=deque(mkf(b),maxlen=tr.max_buffer_frames)
    CAP.clear()
    a=tr.get_angle_for_shot(shot_timestamp=imp,ball_speed_mph=bs,impact_timestamp=impk)
    return (a.vertical_deg if a else None), CAP.get('diag',{})
# ---- baseline pass: capture la_position + nvalid ----
O_UMOD=TR.UMOD_RANGE; O_IMG=TR.IMAGE_MAX_EL_DEG; O_DEMOD=TR._demodulate_frame
basepos={}
for s,bs,b,tmla in work:
    ret,diag=runshot(b,bs,s['imp'],s['impk'])
    basepos[s['ts']]=dict(pos=diag.get('la_position_deg'),sing=diag.get('la_single_frame_deg'),
                          nval=diag.get('n_frames_valid'),ref=diag.get('refusal_reason') or diag.get('refuse'))
# ---- mode A+B pass ----
TR.UMOD_RANGE=(0.6,1.5); TR.IMAGE_MAX_EL_DEG=5.0
def demodB(*a,**k):
    fr=O_DEMOD(*a,**k)
    if fr.valid and not math.isnan(fr.el_image_deg) and abs(fr.el_ball_deg-fr.el_image_deg)<2.5: fr.valid=False
    return fr
TR._demodulate_frame=demodB
abret={}
for s,bs,b,tmla in work:
    ret,_=runshot(b,bs,s['imp'],s['impk']); abret[s['ts']]=ret
TR.UMOD_RANGE=O_UMOD; TR.IMAGE_MAX_EL_DEG=O_IMG; TR._demodulate_frame=O_DEMOD; TR.estimate_two_ray=_orig
# ---- table ----
TMm={s['ts']:tmla for s,bs,b,tmla in work}; LIVE={s['ts']:s['lv'] for s,bs,b,tmla in work}
def f(x): return ('%.1f'%x if isinstance(x,(int,float)) else '   -')
print('  %-9s %5s %7s %8s %12s'%('ts','TM','live','A+B','pos(base)'))
errs_pos=[]; errs_ab=[]
for ts in sorted(TMm):
    bp=basepos[ts]; pos=bp['pos']
    note=''
    if pos is None:
        if bp['sing'] is not None: note='(1-frame: %.1f)'%bp['sing']
        elif bp['ref']: note='(%s)'%bp['ref']
        elif bp['nval'] is not None: note='(nval=%s)'%bp['nval']
    posstr = ('%.1f'%pos if pos is not None else note)
    print('  %-9s %5.1f %7s %8s %12s'%(ts,TMm[ts],f(LIVE[ts]),f(abret[ts]),posstr))
    if pos is not None: errs_pos.append(abs(pos-TMm[ts]))
    if abret[ts] is not None: errs_ab.append(abs(abret[ts]-TMm[ts]))
livee=[abs(LIVE[ts]-TMm[ts]) for ts in TMm if LIVE[ts] is not None]
print('\n  MAE vs TM:  live %.2f (n=%d) | A+B %.2f (n=%d) | la_position(base) %.2f (n=%d, only 2-frame shots)'%(
    sum(livee)/len(livee),len(livee), sum(errs_ab)/len(errs_ab),len(errs_ab), sum(errs_pos)/len(errs_pos),len(errs_pos)))
