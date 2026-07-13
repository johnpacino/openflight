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
MOUNT=10.5; OFFSET=1.5
CLUBS=[('9i','session_20260608_102632_trackman_9i_2.jsonl','compare_9i.csv'),
       ('8i','session_20260608_103725_trackman_8i.jsonl','compare_8i.csv'),
       ('6i','session_20260608_105215_trackman_6i.jsonl','compare_6i.csv'),
       ('5i','session_20260608_110136_trackman_5i.jsonl','compare_5i.csv'),
       ('4i','session_20260608_110913_trackman_4i.jsonl','compare_4i.csv')]
# Standard launch angle by club (AMATEUR / mid-handicap averages, deg) -> gate = standard - 8
# (amateur runs ~+2.5-3 above tour avg; approximate, swappable)
STD_LAUNCH={'pw':26.5,'9i':23.5,'8i':21.0,'7i':19.0,'6i':17.0,'5i':15.0,'4i':13.5}
CAP={}; _orig=TR.estimate_two_ray
def wrapf(frames, it, *a, **k):
    r=_orig(frames, it, *a, **k); CAP['d']=r.diagnostics; return r
TR.estimate_two_ray=wrapf
def mkf(fr): return [KLD7Frame(timestamp=q['timestamp'],radc=(base64.b64decode(q['radc_b64']) if q.get('radc_b64') else None),arrival_timestamp=q.get('arrival_timestamp'),complete_timestamp=q.get('complete_timestamp'),read_duration_ms=q.get('read_duration_ms'),done_frame_number=q.get('done_frame_number')) for q in fr]
def run(fr,imp,impk,bs):
    tr=KLD7Tracker(port=None,orientation='vertical',buffer_seconds=6.0,angle_offset_deg=OFFSET,range_m=5,speed_kmh=100,vertical_estimator='two_ray',mount_tilt_deg=MOUNT,ball_distance_ft=5.0,ball_above_radar_ft=-4.0/12.0,vertical_flight_window_net_distance_ft=10.0)
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
        out.append(dict(ts=s['ts'],tm=float(r['launch_v_tm']),live=(float(r['launch_v_of']) if r.get('launch_v_of') not in (None,'') else None),
            imp=s['imp'],impk=s['impk'],bs=rb[sn],fr=vb[sn]['frames']))
    return sorted(out,key=lambda x:x['ts'])
def pctl(es,q): es=sorted(es); return np.percentile(es,q) if es else float('nan')
print('6/8 remaining clubs — offline cascade (tilt 10.5/off 1.5; gate=STANDARD_launch-8; T2b offset LOO on 6/8):\n')
print('  club  n   TMmed  std   gate  offset | live MAE | cascade: MAE  P50  P90 | tiers')
allcl={}
for club,jf,cf in CLUBS:
    shots=load(jf,cf)
    if not shots: print('  %-4s  (no paired shots)'%club); continue
    tmmed=float(np.median([s['tm'] for s in shots])); std=STD_LAUNCH[club]; gate=std-8.0
    base=[run(s['fr'],s['imp'],s['impk'],s['bs']) for s in shots]
    TR.UMOD_RANGE=(0.6,1.5); TR.IMAGE_MAX_EL_DEG=5.0
    A=[run(s['fr'],s['imp'],s['impk'],s['bs']) for s in shots]
    TR.UMOD_RANGE=O_UMOD; TR.IMAGE_MAX_EL_DEG=O_IMG
    def t1(r): return r['nval']>=2 and r['maxsep']>=9 and r['maxel']>=gate and r['pos'] is not None
    recs=[]
    for i,s in enumerate(shots):
        rb_,ra=base[i],A[i]
        if est(rb_) is None and est(ra) is None: recs.append(('none',None,s['tm'],s['live'])); continue
        if t1(rb_): recs.append(('T1',est(rb_),s['tm'],s['live']))
        elif t1(ra): recs.append(('T2a',est(ra),s['tm'],s['live']))
        else: recs.append(('T2b',est(ra),s['tm'],s['live']))
    # LOO offset for T2b
    t2b=[(i,r) for i,r in enumerate(recs) if r[0]=='T2b' and r[1] is not None]
    yy=[r[1]-r[2] for _,r in t2b]
    off=(-np.mean(yy) if yy else 0.0)
    final=[]
    for i,r in enumerate(recs):
        if r[1] is None: final.append(None); continue
        if r[0]=='T2b':
            others=[yy[j] for j,(ii,_) in enumerate(t2b) if ii!=i]
            corr=-np.mean(others) if others else 0.0
            final.append(r[1]+corr)
        else: final.append(r[1])
    es=[abs(final[i]-recs[i][2]) for i in range(len(recs)) if final[i] is not None]
    live_es=[abs(r[3]-r[2]) for r in recs if r[3] is not None]
    tiers={}
    for r in recs: tiers[r[0]]=tiers.get(r[0],0)+1
    print('  %-4s  %2d  %5.1f  %4.1f  %4.1f  %+4.1f | %6.2f   | %5.2f %4.2f %4.2f | %s'%(
        club,len(shots),tmmed,std,gate,off,(np.mean(live_es) if live_es else 0),np.mean(es),pctl(es,50),pctl(es,90),tiers))
    allcl[club]=(es,live_es)
TR.estimate_two_ray=_orig
alle=[e for es,_ in allcl.values() for e in es]; alll=[e for _,le in allcl.values() for e in le]
print('\n  ALL remaining (9i+8i+6i+5i+4i):  live MAE %.2f | cascade MAE %.2f  P50 %.2f  P90 %.2f  (n=%d)'%(
    np.mean(alll),np.mean(alle),pctl(alle,50),pctl(alle,90),len(alle)))
