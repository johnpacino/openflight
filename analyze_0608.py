import json, numpy as np, math
S=json.load(open('/tmp/of-ttv/cache_0608.json'))
WRAP=5*3.28084; DIST=5.0; ABOVE=-4.0/12.0; MPH=1.4666667
import sys; sys.path[:]=[p for p in sys.path if p not in ('/tmp','/private/tmp','')]; sys.path.insert(0,'/tmp/of-ttv/src')
from openflight.kld7.two_ray import _fit_position_la
clubs=sorted(set(r['club'] for r in S))
GATE={c: float(np.median([r['tm'] for r in S if r['club']==c]))-8.0 for c in clubs}  # John-matched gate (in-sample)
def est(p): return p['pos'] if p['pos'] is not None else (p['sing'] if p['sing'] is not None else p['ret'])
def t1ok(p,gate): return p['nval']>=2 and p['maxsep']>=9 and p['maxel']>=gate and p['pos'] is not None
def dealias_pos(p,bs,netcap_ms=None):
    R=[];E=[];W=[]
    for f in p['frames']:
        if f['rng'] is None: continue
        if netcap_ms is not None and f['t']>netcap_ms: continue
        s=bs*MPH*(f['t']/1000.0); xx=DIST+s*math.cos(math.radians(17)); yy=ABOVE+s*math.sin(math.radians(17)); exp=math.hypot(xx,yy)
        k=max(0,round((exp-f['rng'])/WRAP)); rd=f['rng']+k*WRAP
        if 4.5<=rd<=17.0: R.append(rd);E.append(f['el']);W.append(1/(f['res']+0.02))
    return (_fit_position_la(np.array(R),np.array(E),np.array(W),DIST,ABOVE) if R else None)
def cascade(shift=0.0, curve_sel=False, dealias=False):
    recs=[]
    for r in S:
        b,a=r['base'],r['A']; gate=GATE[r['club']]
        if t1ok(b,gate): tier,e='T1',est(b)
        elif t1ok(a,gate): tier,e='T2a',est(a)
        else: tier,e='T2b',est(a)
        if e is None: recs.append(dict(r=r,tier='none',raw=None)); continue
        # tactic: curve-vs-position when position likely dropped a high-el frame (wrap/band-edge) and they disagree
        if curve_sel and tier!='T2b' and a['pos'] is not None and a['curve'] is not None:
            hi_dropped=any(f['rng'] is not None and f['el']>=gate and (f['rng']<4.5 or f['rng']>15.5) for f in a['frames'])
            if hi_dropped and abs(a['curve']-a['pos'])>3.0: e=a['curve']
        if dealias:
            netcap=1000.0*(10.0)/max(r['bs']*MPH,1)  # ~time to 10ft flight
            dp=dealias_pos(a,r['bs'],netcap_ms=netcap+30)
            if dp is not None and tier!='T1': e=dp
        recs.append(dict(r=r,tier=tier,raw=e+shift))
    return recs
def apply_t2b_offset(recs):  # LOO per club on T2b
    out=[]
    byc={}
    for rc in recs:
        if rc['raw'] is None: out.append(None); continue
        byc.setdefault(rc['r']['club'],[]).append(rc)
    finals={id(rc):None for rc in recs}
    for club,rcs in byc.items():
        t2b=[rc for rc in rcs if rc['tier']=='T2b']
        yy=[rc['raw']-rc['r']['tm'] for rc in t2b]
        for rc in rcs:
            if rc['tier']=='T2b':
                others=[yy[j] for j,o in enumerate(t2b) if o is not rc]
                finals[id(rc)]=rc['raw']-(np.mean(others) if others else 0)
            else: finals[id(rc)]=rc['raw']
    return finals
def score(recs):
    finals=apply_t2b_offset(recs)
    es=[abs(finals[id(rc)]-rc['r']['tm']) for rc in recs if rc['raw'] is not None]
    es=sorted(es); n=len(es); return np.mean(es),np.percentile(es,50),np.percentile(es,90),n,finals
print('=== BASELINE (John-matched gate, T2b LOO offset) ===')
base=cascade()
mae,p50,p90,n,bf=score(base)
print('  ALL n=%d  MAE %.2f  P50 %.2f  P90 %.2f'%(n,mae,p50,p90))
# signed residuals -> session bias
print('\n=== PATTERN: signed residual (est-TM), by tier and club ===')
for tier in ['T1','T2a','T2b']:
    rs=[bf[id(rc)]-rc['r']['tm'] for rc in base if rc['tier']==tier and rc['raw'] is not None]
    if rs: print('  %-4s n=%2d  mean signed %+.2f  (|mean|=session bias if T1)'%(tier,len(rs),np.mean(rs)))
print('  per-club T1 signed residual (the clean-shot bias):')
for c in clubs:
    rs=[bf[id(rc)]-rc['r']['tm'] for rc in base if rc['tier'] in ('T1','T2a') and rc['r']['club']==c and rc['raw'] is not None]
    if rs: print('     %-3s n=%d  %+.2f'%(c,len(rs),np.mean(rs)))
# correlations of |residual| with features
print('\n=== PATTERN: |residual| correlations (all shots) ===')
rr=[(rc,bf[id(rc)]-rc['r']['tm']) for rc in base if rc['raw'] is not None]
res=np.array([e for _,e in rr]); ares=np.abs(res)
bs=np.array([rc['r']['bs'] for rc,_ in rr]); mxel=np.array([rc['r']['A']['maxel'] for rc,_ in rr]); mxsep=np.array([rc['r']['A']['maxsep'] for rc,_ in rr]); nv=np.array([rc['r']['A']['nval'] for rc,_ in rr])
for nm,v in [('ball_speed',bs),('maxel',mxel),('maxsep',mxsep),('nval',nv)]:
    print('  corr(|res|, %-10s) = %+.2f   corr(signed res, %-10s) = %+.2f'%(nm,np.corrcoef(ares,v)[0,1],nm,np.corrcoef(res,v)[0,1]))
# TACTICS
print('\n=== TACTICS (MAE / P50 / P90) ===')
def show(lab,recs):
    mae,p50,p90,n,_=score(recs); print('  %-40s MAE %.2f  P50 %.2f  P90 %.2f'%(lab,mae,p50,p90))
show('T0 baseline',base)
# session-bias shift estimated from T1 clean shots (1 scalar, all clubs)
t1res=[bf[id(rc)]-rc['r']['tm'] for rc in base if rc['tier'] in ('T1','T2a') and rc['raw'] is not None]
sb=-float(np.mean(t1res))
show('T1 +uniform session shift (%.2f)'%sb,cascade(shift=sb))
show('T2 curve-vs-position selection',cascade(curve_sel=True))
show('T3 net-cap-safe de-aliasing',cascade(dealias=True))
# T4: PER-CLUB bias shift (each club by -its T1 mean residual) = per-club calibration ceiling
perclub={}
for c in clubs:
    rs=[bf[id(rc)]-rc['r']['tm'] for rc in base if rc['tier'] in ('T1','T2a') and rc['r']['club']==c and rc['raw'] is not None]
    perclub[c]=-float(np.mean(rs)) if rs else 0.0
def cascade_perclub():
    recs=cascade()
    for rc in recs:
        if rc['raw'] is not None: rc['raw']+=perclub[rc['r']['club']]
    return recs
show('T4 PER-CLUB bias shift (calib ceiling)',cascade_perclub())
# T5: refuse no-valid-frame fallbacks (pos & sing both None -> emit nothing)
def cascade_refuse(**kw):
    recs=cascade(**kw)
    for rc in recs:
        b,a=rc['r']['base'],rc['r']['A']
        if a['pos'] is None and a['sing'] is None and b['pos'] is None and b['sing'] is None:
            rc['raw']=None  # refuse: no usable two_ray frames
    return recs
def score_cov(recs):
    finals=apply_t2b_offset(recs)
    es=[abs(finals[id(rc)]-rc['r']['tm']) for rc in recs if rc['raw'] is not None]
    nref=sum(1 for rc in recs if rc['raw'] is None)
    es=sorted(es); n=len(es); print('  %-40s MAE %.2f  P50 %.2f  P90 %.2f  (n=%d, %d refused)'%(LAB,np.mean(es),np.percentile(es,50),np.percentile(es,90),n,nref))
LAB='T5 refuse no-frame fallbacks'; score_cov(cascade_refuse())
def cascade_refuse_perclub():
    recs=cascade_refuse()
    for rc in recs:
        if rc['raw'] is not None: rc['raw']+=perclub[rc['r']['club']]
    return recs
LAB='T4+T5 per-club shift + refuse fallbacks'; score_cov(cascade_refuse_perclub())
# worst-miss characterization
print('\n=== WORST MISSES (|residual|>3.5) — what do they share? ===')
worst=sorted([(rc,abs(bf[id(rc)]-rc['r']['tm'])) for rc in base if rc['raw'] is not None],key=lambda x:-x[1])[:14]
print('  club ts        TM   est  |res| tier nval maxel maxsep  bs   curve  pos')
for rc,e in worst:
    r=rc['r']; a=r['A']
    print('  %-3s %s %4.1f %5.1f %4.1f  %-4s %d   %4.1f  %5.1f %4.0f  %s %s'%(
        r['club'],r['ts'],r['tm'],bf[id(rc)],e,rc['tier'],a['nval'],a['maxel'],a['maxsep'],r['bs'],
        ('%.1f'%a['curve'] if a['curve'] is not None else ' - '),('%.1f'%a['pos'] if a['pos'] is not None else ' - ')))
