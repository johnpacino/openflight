import sys, json, base64, logging, math, csv
sys.path[:] = [p for p in sys.path if p not in ('/tmp','/private/tmp','')]
sys.path.insert(0, '/tmp/of-ttv/src')
from collections import deque
from datetime import datetime
import numpy as np
logging.disable(logging.WARNING)
from openflight.kld7.tracker import KLD7Tracker
from openflight.kld7.types import KLD7Frame
import openflight.kld7.two_ray as TR
DIR='/Users/john.pacino/openflight_sessions/trackman-6-8'
# 6/8 documented geometry (from kld7_geometry_report config.json)
MOUNT=10.0; OFFSET=2.5; ABOVE=-4.0/12.0; DIST=5.0
# FROZEN 6/15 cascade params
GATE={'7-iron':9,'pw':14}; OFFSET_CORR={'7-iron':3.5,'pw':4.5}
CLUBS={'7-iron':('session_20260608_104504_trackman_7i.jsonl','compare_7i.csv'),
       'pw':('session_20260608_113404_trackman_pw.jsonl','compare_pw.csv')}
def tmmap(csvf):
    m={}
    for r in csv.DictReader(open(DIR+'/'+csvf)):
        v=r.get('launch_v_tm')
        if v in (None,''): continue
        m[r['timestamp_of'][11:19]]=float(v)   # key on HH:MM:SS of OF timestamp
    return m
def mkf(frames): return [KLD7Frame(timestamp=q['timestamp'],radc=(base64.b64decode(q['radc_b64']) if q.get('radc_b64') else None),
    arrival_timestamp=q.get('arrival_timestamp'),complete_timestamp=q.get('complete_timestamp'),read_duration_ms=q.get('read_duration_ms'),done_frame_number=q.get('done_frame_number')) for q in frames]
CAP={}; _orig=TR.estimate_two_ray
def wrap(frames, impact_timestamp, *a, **k):
    r=_orig(frames, impact_timestamp, *a, **k); CAP['d']=r.diagnostics; CAP['la']=r.launch_angle_deg; return r
TR.estimate_two_ray=wrap
def run(b):
    tr=KLD7Tracker(port=None,orientation='vertical',buffer_seconds=6.0,angle_offset_deg=OFFSET,range_m=5,speed_kmh=100,
        vertical_estimator='two_ray',mount_tilt_deg=MOUNT,ball_distance_ft=DIST,ball_above_radar_ft=ABOVE,vertical_flight_window_net_distance_ft=10.0)
    tr._ring_buffer=deque(mkf(b['frames']),maxlen=tr.max_buffer_frames)
    CAP.clear(); ret=tr.get_angle_for_shot(shot_timestamp=b['imp'],ball_speed_mph=b['bs'],impact_timestamp=b['impk']); dg=CAP.get('d',{})
    return dict(ret=(ret.vertical_deg if ret else None),pos=dg.get('la_position_deg'),sing=dg.get('la_single_frame_deg'),
        nval=dg.get('n_frames_valid') or 0,
        maxsep=max([abs(f['el_deg']-f['el_image_deg']) for f in dg.get('frames',[]) if f['el_image_deg'] is not None]+[0.0]),
        maxel=max([f['el_deg'] for f in dg.get('frames',[])]+[0.0]))
def est(r): return r['pos'] if r['pos'] is not None else (r['sing'] if r['sing'] is not None else r['ret'])
O_UMOD=TR.UMOD_RANGE; O_IMG=TR.IMAGE_MAX_EL_DEG
def pct(es,q): es=sorted(es); return np.percentile(es,q) if es else float('nan')
allrecs={}
for club,(jf,cf) in CLUBS.items():
    TM=tmmap(cf); E=GATE[club]; CORR=OFFSET_CORR[club]
    sd={}; vbuf={}; rbs={}
    for l in open(DIR+'/'+jf):
        o=json.loads(l); t=o.get('type'); sn=o.get('shot_number')
        if t=='shot_detected': sd[sn]=dict(ts=o['ts'][11:19],imp=o.get('impact_timestamp'),impk=o.get('impact_timestamp_kld7'))
        elif t=='rolling_buffer_capture': rbs[sn]=o.get('ball_speed_mph')
        elif t=='kld7_buffer' and o.get('orientation')=='vertical': vbuf[sn]=o
    shots=[]
    for sn,s in sd.items():
        if sn not in vbuf or sn not in rbs or s['ts'] not in TM: continue
        shots.append(dict(ts=s['ts'],tm=TM[s['ts']],imp=s['imp'],impk=s['impk'],bs=rbs[sn],frames=vbuf[sn]['frames']))
    shots.sort(key=lambda x:x['ts'])
    base=[run(s) for s in shots]
    TR.UMOD_RANGE=(0.6,1.5); TR.IMAGE_MAX_EL_DEG=5.0
    A=[run(s) for s in shots]
    TR.UMOD_RANGE=O_UMOD; TR.IMAGE_MAX_EL_DEG=O_IMG
    def is_t1(rb): return rb['nval']>=2 and rb['maxsep']>=9 and rb['maxel']>=E and rb['pos'] is not None
    recs=[]
    for i,s in enumerate(shots):
        rb,ra=base[i],A[i]
        if est(rb) is None and est(ra) is None: recs.append(dict(ts=s['ts'],tm=s['tm'],tier='none',final=None)); continue
        if is_t1(rb): recs.append(dict(ts=s['ts'],tm=s['tm'],tier='T1',final=est(rb)))
        elif is_t1(ra): recs.append(dict(ts=s['ts'],tm=s['tm'],tier='T2a',final=est(ra)))
        else: recs.append(dict(ts=s['ts'],tm=s['tm'],tier='T2b',final=est(ra)+CORR))
    allrecs[club]=recs
TR.estimate_two_ray=_orig
def report(name,recs):
    es=[abs(r['final']-r['tm']) for r in recs if r['final'] is not None]
    none=sum(1 for r in recs if r['final'] is None)
    tiers={}
    for r in recs: tiers[r['tier']]=tiers.get(r['tier'],0)+1
    print('  %-9s n=%2d  MAE %.2f  P50 %.2f  P90 %.2f%s   tiers=%s'%(name,len(es),np.mean(es),pct(es,50),pct(es,90),
        ('  [+%d none]'%none if none else ''),tiers))
    # Tier-1 signed bias (calibration check)
    t1=[r['final']-r['tm'] for r in recs if r['tier']=='T1']
    if t1: print('             Tier-1 signed bias (calib check): mean %+.2f  (n=%d) — near 0 = geometry transferred'%(np.mean(t1),len(t1)))
print('=== HELD-OUT 6/8 (frozen 6/15 cascade; 6/8 geometry mount=10.0/offset=2.5) ===\n')
for club in ['pw','7-iron']: report(club,allrecs[club])
report('COMBINED',allrecs['pw']+allrecs['7-iron'])
print('\n  (6/15 in-sample was: PW 1.13/0.56 | 7i 1.23/1.34 | combined 1.18/1.17)')
print('\nPer-shot:')
for club in ['pw','7-iron']:
    print(' '+club)
    for r in sorted(allrecs[club],key=lambda r:r['ts']):
        if r['final'] is None: print('   %s TM %4.1f  none'%(r['ts'],r['tm'])); continue
        print('   %s TM %4.1f  %-4s final %5.1f  err %+5.1f'%(r['ts'],r['tm'],r['tier'],r['final'],r['final']-r['tm']))
