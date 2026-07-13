import sys, json, base64, logging
sys.path[:] = [p for p in sys.path if p not in ('/tmp','/private/tmp','')]
sys.path.insert(0, '/tmp/of-ttv/src')
from collections import deque
from datetime import datetime
import numpy as np
logging.disable(logging.WARNING)
from openflight.kld7.tracker import KLD7Tracker
from openflight.kld7.types import KLD7Frame
from openflight.kld7 import two_ray as TR
SESSION='/Users/john.pacino/openflight_sessions/session_20260615_120512_range.CORRECTED.jsonl'
TMJSON='/Users/john.pacino/openflight_sessions/trackman_06151200_7i_PW_DR.json'
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
h,e=np.histogram(diffs,bins=bn); pk=e[h.argmax()]; off=float(np.median(diffs[(diffs>=pk-6)&(diffs<=pk+6)]))
tmpw=[x for x in tm if x['club']=='pw']
def mkf(b): return [KLD7Frame(timestamp=d['timestamp'],radc=(base64.b64decode(d['radc_b64']) if d.get('radc_b64') else None),
    arrival_timestamp=d.get('arrival_timestamp'),complete_timestamp=d.get('complete_timestamp'),read_duration_ms=d.get('read_duration_ms'),done_frame_number=d.get('done_frame_number')) for d in b['frames']]
def fdict(b):
    out=[]
    for d in b['frames']:
        fd={'timestamp':d['timestamp']}
        if d.get('radc_b64'): fd['radc']=base64.b64decode(d['radc_b64'])
        out.append(fd)
    return out
shots=[]
for sn,s in sd.items():
    if s['club']!='pw' or sn not in vbuf or sn not in rawbs: continue
    dd,best=min(((abs((s['sod']-x['sod'])-off),x) for x in tmpw),key=lambda c:c[0])
    if dd>12: continue
    tr=KLD7Tracker(port=None,orientation='vertical',buffer_seconds=6.0,angle_offset_deg=1.5,range_m=5,speed_kmh=100,vertical_estimator='two_ray',mount_tilt_deg=10.5,ball_distance_ft=5.0,ball_above_radar_ft=-4.0/12.0,vertical_flight_window_net_distance_ft=10.0)
    tr._ring_buffer=deque(mkf(vbuf[sn]),maxlen=tr.max_buffer_frames)
    a=tr.get_angle_for_shot(shot_timestamp=s['imp'],ball_speed_mph=rawbs[sn],impact_timestamp=s['impk']); ang=a.vertical_deg if a else None
    res=TR.estimate_two_ray(fdict(vbuf[sn]), impact_timestamp=s['impk'], ball_speed_mph=rawbs[sn], mount_deg=10.5, angle_offset_deg=1.5, distance_ft=5.0, ball_above_radar_ft=-4.0/12.0, net_distance_ft=10.0, range_m=5.0)
    dg=res.diagnostics; fr=dg.get('frames') or []; far_el=max([f['el_deg'] for f in fr]) if fr else None; nfr=dg.get('n_frames_valid')
    shots.append((s['ts'],ang,far_el,nfr,best['la']))
def mae(es): a=np.abs(np.array(es)); return a.mean(),np.percentile(a,50),np.percentile(a,90)
def corr(fe,nfr,mode):
    if fe is None or fe>=9.5: return 0
    if mode=='flat8': return 8
    if mode=='cond': return 9 if nfr==1 else 5   # single-frame +9, two-frame +5
    return 0
for mode in ['base','flat8','cond']:
    es=[ang+(0 if mode=='base' else corr(fe,nfr,mode))-tmla for ts,ang,fe,nfr,tmla in shots if ang is not None]
    m,p50,p90=mae(es); print('  %-6s: MAE %.2f  P50 %.2f  P90 %.2f'%(mode,m,p50,p90))
print('\n  per-shot (cond: single->+9, two->+5):')
for ts,ang,fe,nfr,tmla in sorted(shots,key=lambda x:x[0]):
    if ang is None: continue
    c=corr(fe,nfr,'cond'); 
    if c: print('   %s nfr=%s far_el=%4.1f  %4.1f(%+5.1f) -> %4.1f(%+5.1f)  +%d'%(ts,nfr,fe,ang,ang-tmla,ang+c,ang+c-tmla,c))
