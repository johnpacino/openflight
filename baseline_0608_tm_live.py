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
MOUNT=10.5; OFFSET=1.5            # boresight 12.0 (matches 6/15)
GATE={'7-iron':9,'pw':14}; OFFc={'7-iron':3.5,'pw':4.5}
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
def cascade_shot(s,club):
    E=GATE[club]; corr=OFFc[club]
    rb=run(s['fr'],s['imp'],s['impk'],s['bs'])
    TR.UMOD_RANGE=(0.6,1.5); TR.IMAGE_MAX_EL_DEG=5.0
    ra=run(s['fr'],s['imp'],s['impk'],s['bs'])
    TR.UMOD_RANGE=O_UMOD; TR.IMAGE_MAX_EL_DEG=O_IMG
    def t1(r): return r['nval']>=2 and r['maxsep']>=9 and r['maxel']>=E and r['pos'] is not None
    if est(rb) is None and est(ra) is None: return None,'none'
    if t1(rb): return est(rb),'T1'
    if t1(ra): return est(ra),'T2a'
    return (est(ra)+corr if est(ra) is not None else None),'T2b'
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
        live=float(r['launch_v_of']) if r.get('launch_v_of') not in (None,'') else None
        out.append(dict(ts=s['ts'],tm=float(r['launch_v_tm']),live=live,imp=s['imp'],impk=s['impk'],bs=rb[sn],fr=vb[sn]['frames']))
    return sorted(out,key=lambda x:x['ts'])
def pctl(es,q): es=sorted(es); return np.percentile(es,q) if es else float('nan')
def report(club,jf,cf):
    shots=load(jf,cf)
    print('\n=== %s  (6/8, tilt 10.5 / offset 1.5) ==='%club.upper())
    print('  ts        TM    live   base  tier | live_err  base_err')
    live_e=[];base_e=[]
    for s in shots:
        base,tier=cascade_shot(s,club)
        le=(s['live']-s['tm']) if s['live'] is not None else None
        be=(base-s['tm']) if base is not None else None
        print('  %s %5.1f  %5s  %5s  %-4s | %7s   %7s'%(s['ts'],s['tm'],
            ('%.1f'%s['live'] if s['live'] is not None else '  -  '),('%.1f'%base if base is not None else '  -  '),tier,
            ('%+.1f'%le if le is not None else '  -  '),('%+.1f'%be if be is not None else '  -  ')))
        if le is not None: live_e.append(abs(le))
        if be is not None: base_e.append(abs(be))
    print('  ---')
    print('  live-logged  vs TM:  MAE %.2f  P50 %.2f  P90 %.2f  (n=%d)'%(np.mean(live_e),pctl(live_e,50),pctl(live_e,90),len(live_e)))
    print('  new baseline vs TM:  MAE %.2f  P50 %.2f  P90 %.2f  (n=%d)'%(np.mean(base_e),pctl(base_e,50),pctl(base_e,90),len(base_e)))
    return live_e,base_e
lp,bp=report('pw','session_20260608_113404_trackman_pw.jsonl','compare_pw.csv')
li,bi=report('7-iron','session_20260608_104504_trackman_7i.jsonl','compare_7i.csv')
TR.estimate_two_ray=_orig
print('\n=== TOTAL (PW+7i) ===')
print('  live-logged  vs TM:  MAE %.2f  P50 %.2f  P90 %.2f  (n=%d)'%(np.mean(lp+li),pctl(lp+li,50),pctl(lp+li,90),len(lp+li)))
print('  new baseline vs TM:  MAE %.2f  P50 %.2f  P90 %.2f  (n=%d)'%(np.mean(bp+bi),pctl(bp+bi,50),pctl(bp+bi,90),len(bp+bi)))
