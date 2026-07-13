import json, numpy as np
recs=json.load(open('/tmp/of-ttv/recs.json'))
def primary(r):
    if r['pos'] is not None: return r['pos']
    if r['sing'] is not None: return r['sing']
    return r['live']
for r in recs:
    p=primary(r); r['prim']=p; r['err']=(abs(p-r['tm']) if p is not None else None)
def is_t1(r): return r['nval']>=2 and r['maxsep']>=12 and r['maxel']>=12 and r['pos'] is not None
def stats(rows):
    es=[r['err'] for r in rows if r['err'] is not None]
    if not es: return '   n=0'
    es=sorted(es); n=len(es)
    return 'n=%2d  MAE %5.2f  P50 %5.2f  P90 %5.2f'%(n,np.mean(es),es[n//2],es[min(n-1,int(0.9*n))])
def line(label,rows):
    t1=[r for r in rows if is_t1(r)]; rej=[r for r in rows if not is_t1(r)]
    none=[r for r in rows if r['err'] is None]
    print('  %-9s  ALL: %s'%(label,stats(rows)))
    print('             TIER1 (maxsep>=12 & maxel>=12): %s'%stats(t1))
    print('             REJECT (everything else):       %s%s'%(stats(rej),
        ('   [+%d no-estimate]'%len(none) if none else '')))
print('=== MAE by bucket (estimator = la_position default) ===\n')
line('ALL CLUBS',recs)
print()
for club in ['7-iron','pw','driver']:
    line(club,[r for r in recs if r['club']==club])
    print()
# also show MAE at the other two operating points, all-clubs and irons+wedges
print('=== how Tier-1 MAE/coverage shifts with the threshold (irons+wedges only) ===')
iw=[r for r in recs if r['club'] in ('7-iron','pw')]
for S,E in [(12,9),(12,12),(15,12)]:
    sel=[r for r in iw if r['nval']>=2 and r['maxsep']>=S and r['maxel']>=E and r['pos'] is not None]
    rej=[r for r in iw if not(r['nval']>=2 and r['maxsep']>=S and r['maxel']>=E and r['pos'] is not None)]
    print('  maxsep>=%2d & maxel>=%2d :  TIER1 %s   ||  REJECT %s'%(S,E,stats(sel),stats(rej)))
