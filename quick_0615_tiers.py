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
S='/Users/john.pacino/openflight_sessions/session_20260615_120512_range.CORRECTED.jsonl'
T='/Users/john.pacino/openflight_sessions/trackman_06151200_7i_PW_DR.json'
GATE={'pw':14,'7-iron':9}
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
def sod(dt): return dt.hour*3600+dt.minute*60+dt.second+dt.microsecond/1e6
sd={};vb={};rb={}
for l in open(S):
    o=json.loads(l);t=o.get('type');sn=o.get('shot_number')
    if t=='shot_detected': sd[sn]=dict(sod=sod(datetime.fromisoformat(o['ts'])),club=o.get('club'),imp=o.get('impact_timestamp'),impk=o.get('impact_timestamp_kld7'))
    elif t=='rolling_buffer_capture': rb[sn]=o.get('ball_speed_mph')
    elif t=='kld7_buffer' and o.get('orientation')=='vertical': vb[sn]=o
d=json.load(open(T))
def load(club,key):
    tms=[]
    for sg in d['StrokeGroups']:
        if sg.get('Club')!=key: continue
        for s in sg['Strokes']:
            m=s.get('Measurement') or {}; tms.append(dict(sod=sod(datetime.fromisoformat(s['Time'])),la=float(m['LaunchAngle'])))
    df=np.array([o['sod']-x['sod'] for o in sd.values() if o['club']==club for x in tms]);bn=np.arange(df.min(),df.max()+2,2.0)
    h,e=np.histogram(df,bins=bn);pk=e[h.argmax()];off=float(np.median(df[(df>=pk-6)&(df<=pk+6)]))
    out=[]
    for sn,s in sd.items():
        if s['club']!=club or sn not in vb or sn not in rb: continue
        dd,best=min(((abs((s['sod']-x['sod'])-off),x) for x in tms),key=lambda c:c[0])
        if dd>12: continue
        out.append(dict(tm=best['la'],imp=s['imp'],impk=s['impk'],bs=rb[sn],fr=vb[sn]['frames']))
    return out
def tiers(club,key):
    shots=load(club,key); E=GATE[club]
    base=[run(s['fr'],s['imp'],s['impk'],s['bs']) for s in shots]
    TR.UMOD_RANGE=(0.6,1.5); TR.IMAGE_MAX_EL_DEG=5.0
    A=[run(s['fr'],s['imp'],s['impk'],s['bs']) for s in shots]
    TR.UMOD_RANGE=O_UMOD; TR.IMAGE_MAX_EL_DEG=O_IMG
    def t1(r): return r['nval']>=2 and r['maxsep']>=9 and r['maxel']>=E and r['pos'] is not None
    recs=[]
    for i,s in enumerate(shots):
        rb_,ra=base[i],A[i]
        if est(rb_) is None and est(ra) is None: recs.append(('none',None,s['tm'])); continue
        if t1(rb_): recs.append(('T1',est(rb_),s['tm']))
        elif t1(ra): recs.append(('T1',est(ra),s['tm']))
        else: recs.append(('T2b',est(ra),s['tm']))
    t2b=[(i,r) for i,r in enumerate(recs) if r[0]=='T2b' and r[1] is not None]
    yy=[r[1]-r[2] for _,r in t2b]
    t1e=[];t2e=[];alle=[]
    for i,r in enumerate(recs):
        if r[1] is None: continue
        if r[0]=='T2b':
            others=[yy[j] for j,(ii,_) in enumerate(t2b) if ii!=i]
            e=abs(r[1]-(np.mean(others) if others else 0)-r[2]); t2e.append(e); alle.append(e)
        else:
            e=abs(r[1]-r[2]); t1e.append(e); alle.append(e)
    return t1e,t2e,alle
def st(es): es=sorted(es); n=len(es); return 'n=%2d MAE %.2f P50 %.2f P90 %.2f'%(n,np.mean(es),np.percentile(es,50),np.percentile(es,90)) if n else 'n=0'
print('6/15 tier breakdown (tilt 10.5/off 1.5; gates PW>=14, 7i>=9; T2b LOO offset):\n')
allt1=[];allt2=[];alla=[]
for club,key in [('pw','PitchingWedge'),('7-iron','7Iron')]:
    t1e,t2e,alle=tiers(club,key); allt1+=t1e;allt2+=t2e;alla+=alle
    print('  %s'%club.upper())
    print('     Tier-1 : %s'%st(t1e))
    print('     Tier-2b: %s'%st(t2e))
    print('     ALL    : %s'%st(alle))
print('\n  COMBINED (PW+7i)')
print('     Tier-1 : %s'%st(allt1))
print('     Tier-2b: %s'%st(allt2))
print('     ALL    : %s'%st(alla))
TR.estimate_two_ray=_orig
