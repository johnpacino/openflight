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
from openflight.kld7.two_ray import _fit_position_la
WRAP=5*3.28084; DIST=5.0; ABOVE=-4.0/12.0; MPH=1.4666667
CAP={}; _orig=TR.estimate_two_ray
def wrap(frames, impact_timestamp, *a, **k):
    r=_orig(frames, impact_timestamp, *a, **k); CAP['d']=r.diagnostics; return r
TR.estimate_two_ray=wrap
def mkf(frames): return [KLD7Frame(timestamp=q['timestamp'],radc=(base64.b64decode(q['radc_b64']) if q.get('radc_b64') else None),
    arrival_timestamp=q.get('arrival_timestamp'),complete_timestamp=q.get('complete_timestamp'),read_duration_ms=q.get('read_duration_ms'),done_frame_number=q.get('done_frame_number')) for q in frames]
def run(frames,imp,impk,bs,mount,offset):
    tr=KLD7Tracker(port=None,orientation='vertical',buffer_seconds=6.0,angle_offset_deg=offset,range_m=5,speed_kmh=100,
        vertical_estimator='two_ray',mount_tilt_deg=mount,ball_distance_ft=DIST,ball_above_radar_ft=ABOVE,vertical_flight_window_net_distance_ft=10.0)
    tr._ring_buffer=deque(mkf(frames),maxlen=tr.max_buffer_frames)
    CAP.clear(); tr.get_angle_for_shot(shot_timestamp=imp,ball_speed_mph=bs,impact_timestamp=impk)
    return CAP.get('d',{})
def exp_range(bs,t_ms,nomLA):
    s=bs*MPH*(t_ms/1000.0)
    x=DIST+s*math.cos(math.radians(nomLA)); y=ABOVE+s*math.sin(math.radians(nomLA))
    return math.hypot(x,y)
def dealias_pos(dg,bs,nomLA,band_hi):
    frs=dg.get('frames',[])
    R=[];E=[];W=[]
    for f in frs:
        r=f['range_ft']
        if r is None: continue
        Rexp=exp_range(bs,f['t_ms'],nomLA)
        k=max(0,round((Rexp-r)/WRAP)); rd=r+k*WRAP
        if 4.5<=rd<=band_hi:
            R.append(rd);E.append(f['el_deg']);W.append(1.0/(f['resid']+0.02))
    if len(R)<1: return None
    return _fit_position_la(np.array(R),np.array(E),np.array(W),DIST,ABOVE)
def stat(es): es=sorted(abs(v) for v in es); n=len(es); return 'n=%d MAE %.2f P50 %.2f'%(n,np.mean(es),es[n//2]) if n else 'n=0'
def load(jf,csvf,tmcol='launch_v_tm'):
    DIR='/Users/john.pacino/openflight_sessions/trackman-6-8'
    TM={r['timestamp_of'][11:19]:float(r[tmcol]) for r in csv.DictReader(open(DIR+'/'+csvf)) if r.get(tmcol) not in (None,'')}
    sd={};vb={};rb={}
    for l in open(DIR+'/'+jf):
        o=json.loads(l);t=o.get('type');sn=o.get('shot_number')
        if t=='shot_detected': sd[sn]=dict(ts=o['ts'][11:19],imp=o.get('impact_timestamp'),impk=o.get('impact_timestamp_kld7'))
        elif t=='rolling_buffer_capture': rb[sn]=o.get('ball_speed_mph')
        elif t=='kld7_buffer' and o.get('orientation')=='vertical': vb[sn]=o
    out=[]
    for sn,s in sd.items():
        if sn in vb and sn in rb and s['ts'] in TM: out.append(dict(ts=s['ts'],tm=TM[s['ts']],imp=s['imp'],impk=s['impk'],bs=rb[sn],frames=vb[sn]['frames']))
    return sorted(out,key=lambda x:x['ts'])
print('=== 6/8 7i: la_position ORIG vs DE-ALIASED (geometry 10.0/2.5, nomLA 17) ===')
print('  ts        TM   bs   orig_pos  dealias_pos  | orig_err  dealias_err')
orig=[];deal=[]
for s in load('session_20260608_104504_trackman_7i.jsonl','compare_7i.csv'):
    dg=run(s['frames'],s['imp'],s['impk'],s['bs'],10.0,2.5)
    op=dg.get('la_position_deg'); dp=dealias_pos(dg,s['bs'],17.0,18.0)
    oe=(op-s['tm']) if op is not None else None; de=(dp-s['tm']) if dp is not None else None
    print('  %s %4.1f %4.0f   %s     %s    | %s   %s'%(s['ts'],s['tm'],s['bs'],
        ('%5.1f'%op if op is not None else '  -  '),('%5.1f'%dp if dp is not None else '  -  '),
        ('%+5.1f'%oe if oe is not None else '  -  '),('%+5.1f'%de if de is not None else '  -  ')))
    if oe is not None: orig.append(oe)
    if de is not None: deal.append(de)
print('  ORIG la_position:    %s'%stat(orig))
print('  DE-ALIASED position: %s'%stat(deal))
# 6/15 PW no-regression: clean ranges -> dealias should be ~no-op
print('\n=== 6/15 PW no-regression check (clean ranges; dealias should match orig) ===')
S15='/Users/john.pacino/openflight_sessions/session_20260615_120512_range.CORRECTED.jsonl'
sd={};vb={};rb={}
for l in open(S15):
    o=json.loads(l);t=o.get('type');sn=o.get('shot_number')
    if t=='shot_detected': sd[sn]=dict(ts=o['ts'][11:19],club=o.get('club'),imp=o.get('impact_timestamp'),impk=o.get('impact_timestamp_kld7'))
    elif t=='rolling_buffer_capture': rb[sn]=o.get('ball_speed_mph')
    elif t=='kld7_buffer' and o.get('orientation')=='vertical': vb[sn]=o
chg=0;tot=0
for sn,s in sd.items():
    if s['club']!='pw' or sn not in vb or sn not in rb: continue
    dg=run(vb[sn]['frames'],s['imp'],s['impk'],rb[sn],10.5,1.5)
    op=dg.get('la_position_deg'); dp=dealias_pos(dg,rb[sn],23.0,18.0)
    if op is not None and dp is not None:
        tot+=1
        if abs(op-dp)>0.5: chg+=1
print('  PW shots where de-alias changed la_position by >0.5deg: %d/%d  (0 = clean no-op, good)'%(chg,tot))
TR.estimate_two_ray=_orig
