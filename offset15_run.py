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
GATE={'7-iron':9,'pw':14}; OFF={'7-iron':3.5,'pw':4.5}
CAP={}; _orig=TR.estimate_two_ray
def wrapf(frames, it, *a, **k):
    r=_orig(frames, it, *a, **k); CAP['d']=r.diagnostics; return r
TR.estimate_two_ray=wrapf
def mkf(fr): return [KLD7Frame(timestamp=q['timestamp'],radc=(base64.b64decode(q['radc_b64']) if q.get('radc_b64') else None),arrival_timestamp=q.get('arrival_timestamp'),complete_timestamp=q.get('complete_timestamp'),read_duration_ms=q.get('read_duration_ms'),done_frame_number=q.get('done_frame_number')) for q in fr]
def run(fr,imp,impk,bs,mount,offset):
    tr=KLD7Tracker(port=None,orientation='vertical',buffer_seconds=6.0,angle_offset_deg=offset,range_m=5,speed_kmh=100,vertical_estimator='two_ray',mount_tilt_deg=mount,ball_distance_ft=5.0,ball_above_radar_ft=-4.0/12.0,vertical_flight_window_net_distance_ft=10.0)
    tr._ring_buffer=deque(mkf(fr),maxlen=tr.max_buffer_frames)
    CAP.clear(); ret=tr.get_angle_for_shot(shot_timestamp=imp,ball_speed_mph=bs,impact_timestamp=impk); d=CAP.get('d',{})
    return dict(ret=(ret.vertical_deg if ret else None),pos=d.get('la_position_deg'),sing=d.get('la_single_frame_deg'),nval=d.get('n_frames_valid') or 0,
        maxsep=max([abs(f['el_deg']-f['el_image_deg']) for f in d.get('frames',[]) if f['el_image_deg'] is not None]+[0.0]),
        maxel=max([f['el_deg'] for f in d.get('frames',[])]+[0.0]))
def est(r): return r['pos'] if r['pos'] is not None else (r['sing'] if r['sing'] is not None else r['ret'])
O_UMOD=TR.UMOD_RANGE; O_IMG=TR.IMAGE_MAX_EL_DEG
def cascade(shots,club,mount,offset):
    E=GATE[club]; corr=OFF[club]
    base=[run(s['fr'],s['imp'],s['impk'],s['bs'],mount,offset) for s in shots]
    TR.UMOD_RANGE=(0.6,1.5); TR.IMAGE_MAX_EL_DEG=5.0
    A=[run(s['fr'],s['imp'],s['impk'],s['bs'],mount,offset) for s in shots]
    TR.UMOD_RANGE=O_UMOD; TR.IMAGE_MAX_EL_DEG=O_IMG
    def t1(r): return r['nval']>=2 and r['maxsep']>=9 and r['maxel']>=E and r['pos'] is not None
    out=[]
    for i,s in enumerate(shots):
        rb,ra=base[i],A[i]
        if est(rb) is None and est(ra) is None: continue
        if t1(rb): out.append(('T1',est(rb)-s['tm']))
        elif t1(ra): out.append(('T2a',est(ra)-s['tm']))
        else: out.append(('T2b',est(ra)+corr-s['tm']))
    return out
def sod(dt): return dt.hour*3600+dt.minute*60+dt.second+dt.microsecond/1e6
def load15(club):
    S='/Users/john.pacino/openflight_sessions/session_20260615_120512_range.CORRECTED.jsonl'
    T='/Users/john.pacino/openflight_sessions/trackman_06151200_7i_PW_DR.json'
    sd={};vb={};rb={}
    for l in open(S):
        o=json.loads(l);t=o.get('type');sn=o.get('shot_number')
        if t=='shot_detected': sd[sn]=dict(sod=sod(datetime.fromisoformat(o['ts'])),club=o.get('club'),imp=o.get('impact_timestamp'),impk=o.get('impact_timestamp_kld7'))
        elif t=='rolling_buffer_capture': rb[sn]=o.get('ball_speed_mph')
        elif t=='kld7_buffer' and o.get('orientation')=='vertical': vb[sn]=o
    d=json.load(open(T));tms=[];key={'7-iron':'7Iron','pw':'PitchingWedge'}[club]
    for sg in d['StrokeGroups']:
        if sg.get('Club')!=key: continue
        for s in sg['Strokes']:
            m=s.get('Measurement') or {}; tms.append(dict(sod=sod(datetime.fromisoformat(s['Time'])),la=float(m['LaunchAngle'])))
    df=np.array([o['sod']-x['sod'] for o in sd.values() if o['club']==club for x in tms]); bnp=np.arange(df.min(),df.max()+2,2.0)
    h,e=np.histogram(df,bins=bnp);pk=e[h.argmax()];offt=float(np.median(df[(df>=pk-6)&(df<=pk+6)]))
    out=[]
    for sn,s in sd.items():
        if s['club']!=club or sn not in vb or sn not in rb: continue
        dd,best=min(((abs((s['sod']-x['sod'])-offt),x) for x in tms),key=lambda c:c[0])
        if dd>12: continue
        out.append(dict(tm=best['la'],imp=s['imp'],impk=s['impk'],bs=rb[sn],fr=vb[sn]['frames']))
    return out
def load08(club):
    DIR='/Users/john.pacino/openflight_sessions/trackman-6-8'
    jf={'7-iron':'session_20260608_104504_trackman_7i.jsonl','pw':'session_20260608_113404_trackman_pw.jsonl'}[club]
    cf={'7-iron':'compare_7i.csv','pw':'compare_pw.csv'}[club]
    TM={r['timestamp_of'][11:19]:float(r['launch_v_tm']) for r in csv.DictReader(open(DIR+'/'+cf)) if r.get('launch_v_tm') not in (None,'')}
    sd={};vb={};rb={}
    for l in open(DIR+'/'+jf):
        o=json.loads(l);t=o.get('type');sn=o.get('shot_number')
        if t=='shot_detected': sd[sn]=dict(ts=o['ts'][11:19],imp=o.get('impact_timestamp'),impk=o.get('impact_timestamp_kld7'))
        elif t=='rolling_buffer_capture': rb[sn]=o.get('ball_speed_mph')
        elif t=='kld7_buffer' and o.get('orientation')=='vertical': vb[sn]=o
    return [dict(tm=TM[s['ts']],imp=s['imp'],impk=s['impk'],bs=rb[sn],fr=vb[sn]['frames']) for sn,s in sd.items() if sn in vb and sn in rb and s['ts'] in TM]
PW15,I715=load15('pw'),load15('7-iron'); PW8,I78=load08('pw'),load08('7-iron')
def summ(label,pwres,i7res):
    def stat(res):
        es=sorted(abs(e) for _,e in res); n=len(es)
        t1b=[e for t,e in res if t=='T1']
        return 'n=%2d MAE %.2f P50 %.2f P90 %.2f  T1bias %+.1f(n%d)'%(n,np.mean(es),np.percentile(es,50),np.percentile(es,90),(np.mean(t1b) if t1b else 0),len(t1b))
    tot=pwres+i7res
    print('  %-26s'%label)
    print('      PW    : %s'%stat(pwres))
    print('      7-iron: %s'%stat(i7res))
    print('      TOTAL : %s'%stat(tot))
print('6/15  (tilt 10.5, offset 1.5):')
summ('',cascade(PW15,'pw',10.5,1.5),cascade(I715,'7-iron',10.5,1.5))
print('\n6/8  OLD (tilt 10.0, offset 2.5):')
summ('',cascade(PW8,'pw',10.0,2.5),cascade(I78,'7-iron',10.0,2.5))
print('\n6/8  NEW (tilt 10.0, offset 1.5  <- KLD7 constant):')
summ('',cascade(PW8,'pw',10.0,1.5),cascade(I78,'7-iron',10.0,1.5))
print('\n6/8  ALT (tilt 10.5, offset 1.5):')
summ('',cascade(PW8,'pw',10.5,1.5),cascade(I78,'7-iron',10.5,1.5))
TR.estimate_two_ray=_orig
