import argparse,json,os
from pathlib import Path
import numpy as np
p=argparse.ArgumentParser();p.add_argument('trial',type=int);a=p.parse_args()
root=Path(__file__).resolve().parents[2]
folder=root/'analysis/yaw_tuning';base=folder/f'trial{a.trial:02d}'
rows=[];bad=[]
for n,line in enumerate(base.with_suffix('.csv').read_text().splitlines()[1:],2):
 try:
  v=list(map(float,line.split(',')))
  if len(v)!=10 or not np.isfinite(v).all():raise ValueError()
  rows.append(v)
 except ValueError:bad.append(n)
d=np.asarray(rows)
if len(d)<20:raise SystemExit('Insufficient telemetry')
c=json.loads(base.with_suffix('.config.json').read_text())
rad2deg=180/np.pi
results=[]
for s in sorted(set(d[:,2].astype(int))):
 if s==0 or s>=5:continue
 q=d[(d[:,2]==s)&(d[:,3]==1)]
 if len(q)<10:continue
 t=(q[:,1]-q[0,1])*.001
 previous=d[(d[:,1]<q[0,1])&(d[:,3]==1)]
 y0=previous[-1,5]*.001 if len(previous) else q[0,5]*.001
 y=q[:,5]*.001;target=q[:,4]*.001;delta=target[0]-y0
 direction=np.sign(delta);progress=(y-y0)/delta
 def crossing(v):
  i=np.where(progress>=v)[0]
  return float(t[i[0]]) if len(i) else None
 err=(target-y)*rad2deg
 outside=np.where(np.abs(err)>2)[0]
 after=int(outside[-1]+1) if len(outside) else 0
 settled=float(t[after]) if after<len(t) and t[-1]-t[after]>=1 else None
 tail=t>=t[-1]-1
 results.append({'segment':int(s),'samples':len(q),'duration_s':float(t[-1]),
                 'start_deg':float(y0*rad2deg),'target_deg':float(target[0]*rad2deg),
                 'time_90percent_s':crossing(.9),'rise_10_90_s':None if crossing(.9) is None or crossing(.1) is None else crossing(.9)-crossing(.1),
                 'overshoot_deg':float(max(0,np.max((y-target)*direction*rad2deg))),
                 'settle_2deg_s':settled,'tail_mean_error_deg':float(err[tail].mean()),
                 'tail_std_deg':float(np.std(y[tail]*rad2deg)),
                 'tail_peak_to_peak_deg':float(np.ptp(y[tail]*rad2deg)),
                 'peak_gyro_radps':float(np.max(np.abs(q[:,6]))*.001),
                 'saturation_fraction':float(np.mean(abs(q[:,7])>=c['limit']*990))})
dt=np.diff(d[:,1])*.001
report={'config':c,'rows':len(d),'bad_lines':bad,'states':sorted(set(d[:,3].astype(int).tolist())),
        'max_gap_s':float(dt.max()),'gaps_over_100ms':int(sum(dt>.1)),
        'nonzero_tx_status_rows':int(sum(d[:,8]!=0)),'max_imu_errors':int(d[:,9].max()),'steps':results}
base.with_suffix('.metrics.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
os.environ.setdefault('MPLCONFIGDIR',str(root/'build/yaw_tuning/.mplconfig'))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
fig,axs=plt.subplots(3,1,figsize=(11,9),sharex=True,constrained_layout=True)
t=(d[:,1]-d[0,1])*.001
plotted_target=np.where(d[:,3]==1,d[:,4],np.nan)
axs[0].plot(t,plotted_target*.001*rad2deg,label='target (active test)');axs[0].plot(t,d[:,5]*.001*rad2deg,label='IMU yaw')
axs[0].set(ylabel='Angle (deg)',title=f'Trial {a.trial}: Kp={c["kp"]}, Ki={c["ki"]}, Kd={c["kd"]}, limit={c["limit"]}')
axs[0].legend();axs[1].plot(t,(plotted_target-d[:,5])*.001*rad2deg);axs[1].set(ylabel='Error (deg)',ylim=(-15,15))
axs[2].plot(t,d[:,6]*.001,label='measured gyro');axs[2].plot(t,d[:,7]*.001,label='command')
axs[2].set(xlabel='Time from first frame (s)',ylabel='Angular rate (rad/s)');axs[2].legend()
fig.savefig(base.with_suffix('.png'),dpi=150);plt.close(fig)
print(json.dumps(report,indent=2))
