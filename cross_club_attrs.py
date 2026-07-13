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
def mkf(b): return [KLD7Frame(timestamp=q['timestamp'],radc=(base64.b64decode(q['radc_b64']) if q.get('radc_b64') else None),
    arrival_timestamp=q.get('arrival_timestamp'),complete_timestamp=q.get('complete_timestamp'),read_duration_ms=q.get('read_duration_ms'),done_frame_number=q.get('done_frame_number')) for q in b['frames']]
def build(club):
    tmc=[x for x in tm if x['club']==club]; w=[]
    for sn,s in sd.items():
        if s['club']!=club or sn not in vbuf or sn not in rawbs: continue
        dd,best=min(((abs((s['sod']-x['sod'])-off),x) for x in tmc),key=lambda c:c[0])
        if dd>12: continue
        w.append((s,rawbs[sn],vbuf[sn],best['la']))
    w.sort(key=lambda x:x[0]['ts']); return w
CAP={}; _orig=TR.estimate_two_ray
def wrap(frames, impact_timestamp, *a, **k):
    r=_orig(frames, impact_timestamp, *a, **k); CAP['diag']=r.diagnostics; CAP['la']=r.launch_angle_deg; CAP['conf']=r.confidence; return r
TR.estimate_two_ray=wrap
def runshot(b,bs,imp,impk):
    tr=KLD7Tracker(port=None,orientation='vertical',buffer_seconds=6.0,angle_offset_deg=1.5,range_m=5,speed_kmh=100,
        vertical_estimator='two_ray',mount_tilt_deg=10.5,ball_distance_ft=5.0,ball_above_radar_ft=-4.0/12.0,vertical_flight_window_net_distance_ft=10.0)
    tr._ring_buffer=deque(mkf(b),maxlen=tr.max_buffer_frames)
    CAP.clear(); ret=tr.get_angle_for_shot(shot_timestamp=imp,ball_speed_mph=bs,impact_timestamp=impk)
    return (ret.vertical_deg if ret else None), CAP.get('diag',{}), CAP.get('conf')
recs=[]
for club in ['7-iron','pw','driver']:
    for s,bs,b,tmla in build(club):
        live,diag,conf=runshot(b,bs,s['imp'],s['impk'])
        frs=diag.get('frames',[])
        seps=[abs(f['el_deg']-f['el_image_deg']) for f in frs if f['el_image_deg'] is not None]
        els=[f['el_deg'] for f in frs]; rhos=[f['rho'] for f in frs]
        cur=diag.get('la_curve_deg'); pos=diag.get('la_position_deg'); sing=diag.get('la_single_frame_deg')
        agree=(abs(cur-pos) if (cur is not None and pos is not None) else None)
        recs.append(dict(club=club,ts=s['ts'],tm=tmla,live=live,cur=cur,pos=pos,sing=sing,
            nval=diag.get('n_frames_valid'),maxsep=(max(seps) if seps else None),minsep=(min(seps) if seps else None),
            maxel=(max(els) if els else None),meanrho=(float(np.mean(rhos)) if rhos else None),conf=conf,agree=agree,
            ref=diag.get('refusal_reason')))
TR.estimate_two_ray=_orig
# best available primary = la_position if present else single-frame else live
def primary(r):
    if r['pos'] is not None: return r['pos'],'pos'
    if r['sing'] is not None: return r['sing'],'sing'
    return r['live'],'live'
print('club    ts        TM   live   curve    pos   sing | nval maxsep maxel rho  conf  agree | primary err')
for r in recs:
    p,src=primary(r); err=(p-r['tm']) if p is not None else None
    fmt=lambda x,w=6,d=1: (('%*.*f'%(w,d,x)) if isinstance(x,(int,float)) else '%*s'%(w,'-'))
    print('%-7s %s %s %s %s %s %s |%s %s %s %s %s %s | %s %s'%(
        r['club'][:7], r['ts'], fmt(r['tm'],5), fmt(r['live'],6), fmt(r['cur'],6), fmt(r['pos'],6), fmt(r['sing'],6),
        fmt(r['nval'],4,0), fmt(r['maxsep'],6), fmt(r['maxel'],5), fmt(r['meanrho'],4,2), fmt(r['conf'],5,2), fmt(r['agree'],6),
        src, ('%+.1f'%err if err is not None else '-')))
# selector: does curve<->position agreement predict accuracy of la_position?
print('\n=== SELECTOR TEST: |la_curve - la_position| (TM-free) vs |la_position - TM| ===')
both=[r for r in recs if r['pos'] is not None and r['cur'] is not None]
for thr in [1.0,1.5,2.0,2.5,3.0,99]:
    sel=[r for r in both if r['agree']<=thr]
    if not sel: continue
    errs=sorted(abs(r['pos']-r['tm']) for r in sel)
    p50=errs[len(errs)//2]; p90=errs[min(len(errs)-1,int(0.9*len(errs)))]
    print('  agree<=%4.1f : n=%2d  la_position |err|  P50 %.2f  P90 %.2f  mean %.2f'%(thr,len(sel),p50,p90,np.mean(errs)))
print('\n=== by club, la_position accuracy where agree<=2.0 (Tier-1 candidate) ===')
for club in ['7-iron','pw','driver']:
    sel=[r for r in both if r['club']==club and r['agree']<=2.0]
    rej=[r for r in both if r['club']==club and r['agree']>2.0]
    if sel:
        es=sorted(abs(r['pos']-r['tm']) for r in sel)
        print('  %-7s TIER1 n=%2d  P50 %.2f  P90 %.2f   |  rejected n=%2d'%(club,len(sel),es[len(es)//2],es[min(len(es)-1,int(0.9*len(es)))],len(rej)))
