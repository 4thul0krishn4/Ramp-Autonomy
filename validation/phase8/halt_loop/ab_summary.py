import re,glob,sys,statistics as st
d=sys.argv[1]
for arm in ('pre','fix'):
    o=[]
    for f in glob.glob(f'{d}/{arm}/*_result.log'):
        n=int(re.search(r'_r(\d+)_result',f).group(1)); t=open(f).read()
        o.append((n,int(re.search(r'halt_count: (\d+)',t).group(1)),re.search(r'overall_verdict: (\w+)',t).group(1),'NOT_REACHED' in re.search(r'descent_verdict: (.*)',t).group(1),float(re.search(r'gt_max_abs_y_descent_m: ([\d.]+)',t).group(1))))
    o.sort(); h=[x[1] for x in o]
    print(f'{d.split("/")[-1]:32s} {arm:3s} n={len(o):2d} HALTs {h} median {st.median(h) if h else None} mean {st.mean(h) if h else 0:.1f} FAIL {sum(1 for x in o if x[2]=="FAIL")} PASS {sum(1 for x in o if x[2]=="PASS")} descent-not-reached {sum(x[3] for x in o)} max|y| {max((x[4] for x in o),default=0):.3f}')
