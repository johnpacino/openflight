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
from openflight.kld7.two_ray import _fit_position_la
WRAP=5*3.28084; DIST=5.0; ABOVE=-4.0/12.0; MPH=1.4666667
S='/Users/john.pacino/openflight_sessions/session_20260615_120512_range.CORRECTED.jsonl'
T='/Users/john.pacino/openflight_sessions/trackman_06151200_7i_PW_DR.json'
def sod(dt): return dt.hour*3600+dt.minute*60+dt.second+dt.microsecond/1e6
sd={};vb={};rb={}
for l in open(S):
    o=json.loads(l);t=o.get('type');sn=o.get('shot_number')
    if t=='shot_detected': sd[sn]=dict(sod=sod(datetime.fromisoformat(o['ts'])),ts=o['ts'][11:19],club=o.get('club'),imp=o.get('impact_timestamp'),impk=o.get('impact_timestamp_kld7'))
    elif t=='rolling_buffer_capture': rb[sn]=o.get('ball_speed_mph')
    elif t=='kld7_buffer' and o.get('orientation')=='vertical': vb[sn]=o
d=json.load(open(T));tms=[]
for sg in d['StrokeGroups']:
    if sg.get('Club')!='7Iron': continue
    for s in sg['Strokes']:
        m=s.get('Measurement') or {}; tms.append(dict(sod=sod(datetime.fromisoformat(s['Time'])),la=float(m['LaunchAngle'])))
df=np.array([o['sod']-x['sod'] for o in sd.values() if o['club']=='7-iron' for x in tms]);bnp=np.arange(df.min(),df.max()+2,2.0)
h,e=np.histogram(df,bins=bnp);pk=e[h.argmax()];offt=float(np.median(df[(df>=pk-6)&(df<=pk+6)]))
shots=[]
for sn,s in sd.items():
    if s['club']!='7-iron' or sn not in vb or sn not in rb: continue
    dd,best=min(((abs((s['sod']-x['sod'])-offt),x) for x in tms),key=lambda c:c[0])
    if dd>12: continue
    shots.append(dict(ts=s['ts'],tm=best['la'],imp=s['imp'],impk=s['impk'],bs=rb[sn],fr=vb[sn]['frames']))
shots.sort(key=lambda x:x['ts'])
CAP={};_orig=TR.estimate_two_ray
def wr(f,it,*a,**k):
    r=_orig(f,it,*a,**k);CAP['d']=r.diagnostics;return r
TR.estimate_two_ray=wr
def mkf(fr): return [KLD7Frame(timestamp=q['timestamp'],radc=(base64.b64decode(q['radc_b64']) if q.get('radc_b64') else None),arrival_timestamp=q.get('arrival_timestamp'),complete_timestamp=q.get('complete_timestamp'),read_duration_ms=q.get('read_duration_ms'),done_frame_number=q.get('done_frame_number')) for q in fr]
def run(s):
    tr=KLD7Tracker(port=None,orientation='vertical',buffer_seconds=6.0,angle_offset_deg=1.5,range_m=5,speed_kmh=100,vertical_estimator='two_ray',mount_tilt_deg=10.5,ball_distance_ft=DIST,ball_above_radar_ft=ABOVE,vertical_flight_window_net_distance_ft=10.0)
    tr._ring_buffer=deque(mkf(s['fr']),maxlen=tr.max_buffer_frames)
    CAP.clear();tr.get_angle_for_shot(shot_timestamp=s['imp'],ball_speed_mph=s['bs'],impact_timestamp=s['impk']);return CAP.get('d',{})
def exp_range(bs,t_ms,nom=17.0):
    s=bs*MPH*(t_ms/1000.0);x=DIST+s*math.cos(math.radians(nom));y=ABOVE+s*math.sin(math.radians(nom));return math.hypot(x,y)
def dealias_pos(d,bs):
    R=[];E=[];W=[]
    for f in d.get('frames',[]):
        r=f['range_ft']
        if r is None: continue
        k=max(0,round((exp_range(bs,f['t_ms'])-r)/WRAP)); rd=r+k*WRAP
        if 4.5<=rd<=18.0: R.append(rd);E.append(f['el_deg']);W.append(1/(f['resid']+0.02))
    return (_fit_position_la(np.array(R),np.array(E),np.array(W),DIST,ABOVE) if R else None)
print('6/15 7i — do any shots have wrapped frames? (range raw vs whether de-alias changes la_position)\n')
print('  ts        TM   frame ranges (raw)                | orig_pos  dealias_pos  delta')
nwrap=0
for s in shots:
    d=run(s)
    ranges=[f['range_ft'] for f in d.get('frames',[]) if f['range_ft'] is not None]
    op=d.get('la_position_deg'); dp=dealias_pos(d,s['bs'])
    delta=(dp-op) if (op is not None and dp is not None) else None
    wrapped=any(r<2.5 for r in ranges) or any(r>15.5 for r in ranges)
    if delta is not None and abs(delta)>0.3: nwrap+=1
    print('  %s %4.1f   %-34s | %s    %s    %s%s'%(s['ts'],s['tm'],
        str([round(r,1) for r in ranges]),
        ('%.1f'%op if op is not None else ' - '),('%.1f'%dp if dp is not None else ' - '),
        ('%+.1f'%delta if delta is not None else ' - '),'  <-WRAP' if wrapped else ''))
TR.estimate_two_ray=_orig
print('\n  shots where de-alias changes la_position by >0.3deg: %d/%d'%(nwrap,len(shots)))
