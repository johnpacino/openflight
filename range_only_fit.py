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
D=5.0; ABOVE=-4.0/12.0; WRAP=5*3.28084
CLUBS=[('pw','session_20260608_113404_trackman_pw.jsonl','compare_pw.csv'),
       ('9i','session_20260608_102632_trackman_9i_2.jsonl','compare_9i.csv'),
       ('8i','session_20260608_103725_trackman_8i.jsonl','compare_8i.csv'),
       ('7i','session_20260608_104504_trackman_7i.jsonl','compare_7i.csv'),
       ('6i','session_20260608_105215_trackman_6i.jsonl','compare_6i.csv'),
       ('5i','session_20260608_110136_trackman_5i.jsonl','compare_5i.csv'),
       ('4i','session_20260608_110913_trackman_4i.jsonl','compare_4i.csv')]
# capture every FrameDemod's sub_ranges
CAP={'demods':[]}; _od=TR._demodulate_frame
def wrapd(*a,**k):
    fr=_od(*a,**k); CAP['demods'].append(fr); return fr
TR._demodulate_frame=wrapd
def mkf(fr): return [KLD7Frame(timestamp=q['timestamp'],radc=(base64.b64decode(q['radc_b64']) if q.get('radc_b64') else None),arrival_timestamp=q.get('arrival_timestamp'),complete_timestamp=q.get('complete_timestamp'),read_duration_ms=q.get('read_duration_ms'),done_frame_number=q.get('done_frame_number')) for q in fr]
def subranges(fr,imp,impk,bs):
    tr=KLD7Tracker(port=None,orientation='vertical',buffer_seconds=6.0,angle_offset_deg=1.5,range_m=5,speed_kmh=100,vertical_estimator='two_ray',mount_tilt_deg=10.5,ball_distance_ft=D,ball_above_radar_ft=ABOVE,vertical_flight_window_net_distance_ft=10.0)
    tr._ring_buffer=deque(mkf(fr),maxlen=tr.max_buffer_frames)
    CAP['demods']=[]; tr.get_angle_for_shot(shot_timestamp=imp,ball_speed_mph=bs,impact_timestamp=impk)
    pts=[]
    for d in CAP['demods']:
        for (t,r) in d.sub_ranges: pts.append((t,r))
    return sorted(pts)
def unwrap(pts):  # continuity unwrap: ball range increases monotonically
    if not pts: return []
    out=[]; w=0; prev=None
    for t,r in pts:
        if prev is not None and (r+w*WRAP) < prev-8.0: w+=1
        ru=r+w*WRAP; out.append((t,ru)); prev=ru
    return out
def fit_la(pts):  # grid LA; for each, map R->s (tee-ray), fit line s vs t, min residual
    if len(pts)<6: return None,None,None
    t=np.array([p[0] for p in pts]); R=np.array([p[1] for p in pts])
    best=(1e9,None,None)
    for LA in np.arange(0.0,45.0,0.25):
        c=math.cos(math.radians(LA)); sn=math.sin(math.radians(LA))
        b=D*c+ABOVE*sn; cc=D*D+ABOVE*ABOVE-R*R; disc=b*b-cc
        if np.any(disc<0): continue
        s=-b+np.sqrt(disc)
        if np.any(s<=0): continue
        A=np.vstack([t,np.ones_like(t)]).T
        coef,res,*_=np.linalg.lstsq(A,s,rcond=None)
        resid=np.sum((s-A@coef)**2)
        if resid<best[0]: best=(resid,LA,coef[0]*1000.0)  # slope ft/s
    return best[1],best[2],len(pts)
TM={}; SHOTS=[]
for club,jf,cf in CLUBS:
    rows={r['timestamp_of'][11:19]:r for r in csv.DictReader(open(DIR+'/'+cf))}
    sd={};vb={};rb={}
    for l in open(DIR+'/'+jf):
        o=json.loads(l);t=o.get('type');sn=o.get('shot_number')
        if t=='shot_detected': sd[sn]=dict(ts=o['ts'][11:19],imp=o.get('impact_timestamp'),impk=o.get('impact_timestamp_kld7'))
        elif t=='rolling_buffer_capture': rb[sn]=o.get('ball_speed_mph')
        elif t=='kld7_buffer' and o.get('orientation')=='vertical': vb[sn]=o
    for sn,s in sd.items():
        if sn not in vb or sn not in rb or s['ts'] not in rows or rows[s['ts']].get('launch_v_tm') in (None,''): continue
        SHOTS.append(dict(club=club,ts=s['ts'],tm=float(rows[s['ts']]['launch_v_tm']),bs=rb[sn],imp=s['imp'],impk=s['impk'],fr=vb[sn]['frames']))
res=[]
for s in SHOTS:
    pts=unwrap(subranges(s['fr'],s['imp'],s['impk'],s['bs']))
    la,vfit,n=fit_la(pts)
    res.append(dict(s=s,la=la,vfit=vfit,n=n))
TR._demodulate_frame=_od
def stat(rows): es=sorted(abs(r['la']-r['s']['tm']) for r in rows if r['la'] is not None); n=len(es); return ('n=%2d MAE %.2f P50 %.2f P90 %.2f'%(n,np.mean(es),np.percentile(es,50),np.percentile(es,90))) if n else 'n=0'
print('RANGE-ONLY launch angle (sub-frame range track + tee geometry, NO elevation/boresight):\n')
print('  per club:')
for club in [c[0] for c in CLUBS]:
    rows=[r for r in res if r['s']['club']==club]
    none=sum(1 for r in rows if r['la'] is None)
    print('    %-3s %s   (%d no-fit)'%(club,stat(rows),none))
allr=[r for r in res]
print('  ALL: %s'%stat(allr))
print('\n  sample shots (club ts TM range-LA n_pts vfit bs):')
for r in sorted(res,key=lambda r:r['s']['club'])[:18]:
    s=r['s']
    print('    %-3s %s TM %4.1f  rLA %s  n=%s vfit=%s bs=%.0f'%(s['club'],s['ts'],s['tm'],
        ('%5.1f'%r['la'] if r['la'] is not None else '  -  '),(r['n'] if r['n'] else '-'),
        ('%.0f'%r['vfit'] if r['vfit'] else '-'),s['bs']))
