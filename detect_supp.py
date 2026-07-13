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
    r=_orig(frames, impact_timestamp, *a, **k); CAP['diag']=r.diagnostics; CAP['la']=r.launch_angle_deg; return r
TR.estimate_two_ray=wrap
def runshot(b,bs,imp,impk):
    tr=KLD7Tracker(port=None,orientation='vertical',buffer_seconds=6.0,angle_offset_deg=1.5,range_m=5,speed_kmh=100,
        vertical_estimator='two_ray',mount_tilt_deg=10.5,ball_distance_ft=5.0,ball_above_radar_ft=-4.0/12.0,vertical_flight_window_net_distance_ft=10.0)
    tr._ring_buffer=deque(mkf(b),maxlen=tr.max_buffer_frames)
    CAP.clear(); tr.get_angle_for_shot(shot_timestamp=imp,ball_speed_mph=bs,impact_timestamp=impk)
    return CAP.get('la'),CAP.get('diag',{})
print('  %-9s %5s %6s %6s | per-frame  el / el_img / sep / rho / resid'%('ts','TM','LA','err'))
rec=[]
for s,bs,b,tmla in work:
    la,diag=runshot(b,bs,s['imp'],s['impk']); err=(la-tmla) if la is not None else None
    frs=diag.get('frames',[])
    seps=[]; rhos=[]
    cell=[]
    for fr in frs:
        ei=fr['el_image_deg']; sep=(abs(fr['el_deg']-ei) if ei is not None else float('nan')); rho=fr['rho']
        if not math.isnan(sep): seps.append(sep)
        rhos.append(rho)
        cell.append('%.1f/%s/%s/%.2f/%.3f'%(fr['el_deg'],('%.1f'%ei if ei is not None else 'nan'),
                    ('%.1f'%sep if not math.isnan(sep) else 'nan'),rho,fr['resid']))
    cls='UNDER' if (err is not None and err<-1.5) else ('OVER' if (err is not None and err>1.5) else 'ok')
    print('  %-9s %5.1f %6s %6s %-5s %s'%(s['ts'],tmla,('%.1f'%la if la else '-'),('%+.1f'%err if err is not None else '-'),cls,'  ||  '.join(cell)))
    if err is not None: rec.append(dict(cls=cls,err=err,minsep=(min(seps) if seps else None),meansep=(np.mean(seps) if seps else None),maxrho=max(rhos) if rhos else None,meanrho=np.mean(rhos) if rhos else None))
TR.estimate_two_ray=_orig
print('\n  === aggregate by class (min sep across frames, mean rho) ===')
for c in ['UNDER','ok','OVER']:
    g=[r for r in rec if r['cls']==c]
    if not g: continue
    ms=[r['minsep'] for r in g if r['minsep'] is not None]; mr=[r['meanrho'] for r in g if r['meanrho'] is not None]
    print('  %-6s n=%2d   min-sep(deg): mean %.2f  range %.1f-%.1f   |   mean-rho: mean %.2f range %.2f-%.2f'%(
        c,len(g), np.mean(ms),min(ms),max(ms), np.mean(mr),min(mr),max(mr)))
