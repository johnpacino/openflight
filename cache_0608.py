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
CLUBS=[('pw','session_20260608_113404_trackman_pw.jsonl','compare_pw.csv'),
       ('9i','session_20260608_102632_trackman_9i_2.jsonl','compare_9i.csv'),
       ('8i','session_20260608_103725_trackman_8i.jsonl','compare_8i.csv'),
       ('7i','session_20260608_104504_trackman_7i.jsonl','compare_7i.csv'),
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
    return dict(pos=d.get('la_position_deg'),sing=d.get('la_single_frame_deg'),curve=d.get('la_curve_deg'),ret=(ret.vertical_deg if ret else None),
        nval=d.get('n_frames_valid') or 0,tau=d.get('tau_range_ms'),
        frames=[{'t':f['t_ms'],'el':f['el_deg'],'eli':f['el_image_deg'],'rng':f['range_ft'],'res':f['resid']} for f in d.get('frames',[])])
def feats(r):
    seps=[abs(f['el']-f['eli']) for f in r['frames'] if f['eli'] is not None]
    return dict(maxsep=max(seps) if seps else 0.0,maxel=max([f['el'] for f in r['frames']]+[0.0]))
O_UMOD=TR.UMOD_RANGE; O_IMG=TR.IMAGE_MAX_EL_DEG
out=[]
for club,jf,cf in CLUBS:
    rows={r['timestamp_of'][11:19]:r for r in csv.DictReader(open(DIR+'/'+cf))}
    sd={};vb={};rb={}
    for l in open(DIR+'/'+jf):
        o=json.loads(l);t=o.get('type');sn=o.get('shot_number')
        if t=='shot_detected': sd[sn]=dict(ts=o['ts'][11:19],imp=o.get('impact_timestamp'),impk=o.get('impact_timestamp_kld7'))
        elif t=='rolling_buffer_capture': rb[sn]=o.get('ball_speed_mph')
        elif t=='kld7_buffer' and o.get('orientation')=='vertical': vb[sn]=o
    for sn,s in sd.items():
        if sn not in vb or sn not in rb or s['ts'] not in rows: continue
        r=rows[s['ts']]
        if r.get('launch_v_tm') in (None,''): continue
        b=run(vb[sn]['frames'],s['imp'],s['impk'],rb[sn])
        TR.UMOD_RANGE=(0.6,1.5); TR.IMAGE_MAX_EL_DEG=5.0
        a=run(vb[sn]['frames'],s['imp'],s['impk'],rb[sn])
        TR.UMOD_RANGE=O_UMOD; TR.IMAGE_MAX_EL_DEG=O_IMG
        bf=feats(b); af=feats(a)
        out.append(dict(club=club,ts=s['ts'],tm=float(r['launch_v_tm']),bs=rb[sn],
            live=(float(r['launch_v_of']) if r.get('launch_v_of') not in (None,'') else None),
            base=dict(b,**bf),A=dict(a,**af)))
TR.estimate_two_ray=_orig
json.dump(out,open('/tmp/of-ttv/cache_0608.json','w'))
print('cached %d shots across %d clubs -> cache_0608.json'%(len(out),len(set(r['club'] for r in out))))
for club in [c[0] for c in CLUBS]:
    print('  %s: %d'%(club,sum(1 for r in out if r['club']==club)))
