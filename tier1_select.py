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
D=5.0; ABOVE=-4.0/12.0
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
def mkf(b): return [KLD7Frame(timestamp=q['timestamp'],radc=(base64.b64decode(q['radc_b64']) if q.get('radc_b64') else None),
    arrival_timestamp=q.get('arrival_timestamp'),complete_timestamp=q.get('complete_timestamp'),read_duration_ms=q.get('read_duration_ms'),done_frame_number=q.get('done_frame_number')) for q in b['frames']]
def build(club):
    tmc=[x for x in tm if x['club']==club]; w=[]
    for sn,s in sd.items():
        if s['club']!=club or sn not in vbuf or sn not in rawbs: continue
        dd,best=min(((abs((s['sod']-x['sod'])-off),x) for x in tmc),key=lambda c:c[0])
        if dd>12: continue
        w.append((s,rawbs[sn],vbuf[sn],best['la']))
    return w
CAP={}; _orig=TR.estimate_two_ray
def wrap(frames, impact_timestamp, *a, **k):
    r=_orig(frames, impact_timestamp, *a, **k); CAP['diag']=r.diagnostics; CAP['la']=r.launch_angle_deg; CAP['conf']=r.confidence; return r
TR.estimate_two_ray=wrap
def runshot(b,bs,imp,impk):
    tr=KLD7Tracker(port=None,orientation='vertical',buffer_seconds=6.0,angle_offset_deg=1.5,range_m=5,speed_kmh=100,
        vertical_estimator='two_ray',mount_tilt_deg=10.5,ball_distance_ft=5.0,ball_above_radar_ft=ABOVE,vertical_flight_window_net_distance_ft=10.0)
    tr._ring_buffer=deque(mkf(b),maxlen=tr.max_buffer_frames)
    CAP.clear(); ret=tr.get_angle_for_shot(shot_timestamp=imp,ball_speed_mph=bs,impact_timestamp=impk)
    return (ret.vertical_deg if ret else None), CAP.get('diag',{})
recs=[]
for club in ['7-iron','pw','driver']:
    for s,bs,b,tmla in build(club):
        live,diag=runshot(b,bs,s['imp'],s['impk'])
        frs=diag.get('frames',[])
        seps=[abs(f['el_deg']-f['el_image_deg']) for f in frs if f['el_image_deg'] is not None]
        els=[f['el_deg'] for f in frs]
        recs.append(dict(club=club,ts=s['ts'],tm=tmla,live=live,
            pos=diag.get('la_position_deg'),sing=diag.get('la_single_frame_deg'),cur=diag.get('la_curve_deg'),
            nval=diag.get('n_frames_valid') or 0,maxsep=(max(seps) if seps else 0.0),
            maxel=(max(els) if els else 0.0),bs=bs))
TR.estimate_two_ray=_orig
json.dump(recs, open('/tmp/of-ttv/recs.json','w'))
def primary(r):  # user's default = la_position when present
    if r['pos'] is not None: return r['pos']
    if r['sing'] is not None: return r['sing']
    return r['live']
for r in recs:
    p=primary(r); r['err']=(abs(p-r['tm']) if p is not None else None); r['prim']=p
ok=[r for r in recs if r['err'] is not None]
def pct(v,q): v=sorted(v); return v[min(len(v)-1,int(q*len(v)))]
print('=== attribute means by accuracy tier (primary=la_position default) ===')
for lab,lo,hi in [('within 1.5',0,1.5),('1.5-3',1.5,3),('>3 (bad)',3,99)]:
    g=[r for r in ok if lo<=r['err']<hi]
    if not g: continue
    print('  %-12s n=%2d   maxel %4.1f   maxsep %5.1f   nval %.1f   %%pos %d'%(
        lab,len(g),np.mean([r['maxel'] for r in g]),np.mean([r['maxsep'] for r in g]),
        np.mean([r['nval'] for r in g]),100*sum(1 for r in g if r['pos'] is not None)//len(g)))
print('\n=== Tier-1 selector sweep: require nval>=2 AND maxsep>=S AND maxel>=E, use la_position ===')
for S,E in [(12,9),(12,12),(15,12),(12,14),(15,14)]:
    sel=[r for r in ok if r['nval']>=2 and r['maxsep']>=S and r['maxel']>=E and r['pos'] is not None]
    rej=[r for r in ok if not(r['nval']>=2 and r['maxsep']>=S and r['maxel']>=E and r['pos'] is not None)]
    if not sel: continue
    es=[abs(r['pos']-r['tm']) for r in sel]
    print('  maxsep>=%2d & maxel>=%2d : Tier1 n=%2d  P50 %.2f  P90 %.2f  | rejected n=%2d'%(S,E,len(sel),pct(es,.5),pct(es,.9),len(rej)))
print('\n=== chosen selector (maxsep>=12 & maxel>=12) by club ===')
S,E=12,12
for club in ['7-iron','pw','driver']:
    sel=[r for r in ok if r['club']==club and r['nval']>=2 and r['maxsep']>=S and r['maxel']>=E and r['pos'] is not None]
    allc=[r for r in ok if r['club']==club]
    if sel:
        es=[abs(r['pos']-r['tm']) for r in sel]
        print('  %-7s Tier1 n=%2d/%2d  P50 %.2f  P90 %.2f  mean %.2f'%(club,len(sel),len(allc),pct(es,.5),pct(es,.9),np.mean(es)))
    else:
        print('  %-7s Tier1 n= 0/%2d'%(club,len(allc)))
