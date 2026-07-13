import sys, json, base64, logging, math, csv
sys.path[:] = [p for p in sys.path if p not in ('/tmp','/private/tmp','')]
sys.path.insert(0, '/tmp/of-ttv/src')
from collections import deque
import numpy as np
logging.disable(logging.WARNING)
from openflight.kld7.tracker import KLD7Tracker
from openflight.kld7.types import KLD7Frame
import openflight.kld7.two_ray as TR
DIR='/Users/john.pacino/openflight_sessions/trackman-6-8'
TOUR={'9i':20.4,'8i':18.1,'6i':14.1,'5i':12.1,'4i':11.0}
CLUBS=[('9i','session_20260608_102632_trackman_9i_2.jsonl','compare_9i.csv'),
       ('8i','session_20260608_103725_trackman_8i.jsonl','compare_8i.csv'),
       ('6i','session_20260608_105215_trackman_6i.jsonl','compare_6i.csv'),
       ('5i','session_20260608_110136_trackman_5i.jsonl','compare_5i.csv'),
       ('4i','session_20260608_110913_trackman_4i.jsonl','compare_4i.csv')]
CAP={}; _orig=TR.estimate_two_ray
def wrapf(frames, it, *a, **k):
    r=_orig(frames, it, *a, **k); CAP['d']=r.diagnostics; return r
TR.estimate_two_ray=wrapf
def mkf(fr): return [KLD7Frame(timestamp=q['timestamp'],radc=(base64.b64decode(q['radc_b64']) if q.get('radc_b64') else None),arrival_timestamp=q.get('arrival_timestamp'),complete_timestamp=q.get('complete_timestamp'),read_duration_ms=q.get('read_duration_ms'),done_frame_number=q.get('done_frame_number')) for q in fr]
def run(fr,imp,impk,bs):
    tr=KLD7Tracker(port=None,orientation='vertical',buffer_seconds=6.0,angle_offset_deg=1.5,range_m=5,speed_kmh=100,vertical_estimator='two_ray',mount_tilt_deg=10.5,ball_distance_ft=5.0,ball_above_radar_ft=-4.0/12.0,vertical_flight_window_net_distance_ft=10.0)
    tr._ring_buffer=deque(mkf(fr),maxlen=tr.max_buffer_frames)
    CAP.clear(); ret=tr.get_angle_for_shot(shot_timestamp=imp,ball_speed_mph=bs,impact_timestamp=impk); d=CAP.get('d',{})
    return dict(pos=d.get('la_position_deg'),sing=d.get('la_single_frame_deg'),ret=(ret.vertical_deg if ret else None),nval=d.get('n_frames_valid') or 0,
        maxsep=max([abs(f['el_deg']-f['el_image_deg']) for f in d.get('frames',[]) if f['el_image_deg'] is not None]+[0.0]),
        maxel=max([f['el_deg'] for f in d.get('frames',[])]+[0.0]))
def est(r): return r['pos'] if r['pos'] is not None else (r['sing'] if r['sing'] is not None else r['ret'])
O_UMOD=TR.UMOD_RANGE; O_IMG=TR.IMAGE_MAX_EL_DEG
def load(jf,cf):
    rows={r['timestamp_of'][11:19]:r for r in csv.DictReader(open(DIR+'/'+cf))}
    sd={};vb={};rb={}
    for l in open(DIR+'/'+jf):
        o=json.loads(l);t=o.get('type');sn=o.get('shot_number')
        if t=='shot_detected': sd[sn]=dict(ts=o['ts'][11:19],imp=o.get('impact_timestamp'),impk=o.get('impact_timestamp_kld7'))
        elif t=='rolling_buffer_capture': rb[sn]=o.get('ball_speed_mph')
        elif t=='kld7_buffer' and o.get('orientation')=='vertical': vb[sn]=o
    out=[]
    for sn,s in sd.items():
        if sn not in vb or sn not in rb or s['ts'] not in rows: continue
        r=rows[s['ts']]
        if r.get('launch_v_tm') in (None,''): continue
        out.append(dict(tm=float(r['launch_v_tm']),imp=s['imp'],impk=s['impk'],bs=rb[sn],fr=vb[sn]['frames']))
    return out
# cache replays once per shot
cache={}
for club,jf,cf in CLUBS:
    shots=load(jf,cf); rows=[]
    for s in shots:
        rb_=run(s['fr'],s['imp'],s['impk'],s['bs'])
        TR.UMOD_RANGE=(0.6,1.5); TR.IMAGE_MAX_EL_DEG=5.0
        ra=run(s['fr'],s['imp'],s['impk'],s['bs'])
        TR.UMOD_RANGE=O_UMOD; TR.IMAGE_MAX_EL_DEG=O_IMG
        rows.append((s['tm'],rb_,ra))
    cache[club]=rows
TR.estimate_two_ray=_orig
def evaln(delta):
    allerr=[]; t1err=[]; t2err=[]; nt1=nt2b=nnone=ntot=0
    for club,rows in cache.items():
        gate=(TOUR[club]+delta)-8.0
        recs=[]
        for tm,rb_,ra in rows:
            ntot+=1
            def t1(r): return r['nval']>=2 and r['maxsep']>=9 and r['maxel']>=gate and r['pos'] is not None
            if est(rb_) is None and est(ra) is None: recs.append(('none',None,tm)); nnone+=1; continue
            if t1(rb_): recs.append(('T1',est(rb_),tm)); nt1+=1
            elif t1(ra): recs.append(('T1',est(ra),tm)); nt1+=1
            else: recs.append(('T2b',est(ra),tm)); nt2b+=1
        t2b=[(i,r) for i,r in enumerate(recs) if r[0]=='T2b' and r[1] is not None]
        yy=[r[1]-r[2] for _,r in t2b]
        for i,r in enumerate(recs):
            if r[1] is None: continue
            if r[0]=='T2b':
                others=[yy[j] for j,(ii,_) in enumerate(t2b) if ii!=i]
                e=abs(r[1]-(np.mean(others) if others else 0)-r[2]); allerr.append(e); t2err.append(e)
            else:
                e=abs(r[1]-r[2]); allerr.append(e); t1err.append(e)
    return nt1,nt2b,nnone,ntot,np.mean(allerr),np.percentile(allerr,50),np.percentile(allerr,90),(np.mean(t1err) if t1err else 0),(np.mean(t2err) if t2err else 0)
print('Gate-level sweep (table = tour + delta), remaining 5 clubs, n=71:\n')
print('  table              delta  Tier-1  T2b  none |  MAE   P50   P90 | T1-only MAE  T2b-only MAE')
for lab,delta in [('tour avg',0.0),('YOUR tier (~low-am)',1.5),('mid-amateur',3.0)]:
    nt1,nt2b,nnone,ntot,mae,p50,p90,t1m,t2m=evaln(delta)
    print('  %-18s %+4.1f   %2d/%2d   %2d   %2d  | %5.2f %5.2f %5.2f | %8.2f   %8.2f'%(lab,delta,nt1,ntot,nt2b,nnone,mae,p50,p90,t1m,t2m))
