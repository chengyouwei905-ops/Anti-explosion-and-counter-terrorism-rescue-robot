"""Known 70-mm bodies of revolution: silhouette fit, no table or bbox depth.

The observation is the axis midpoint. Axial rotation is unobservable and is
never reported. The complete object silhouette is required, not the table.
"""
import math
from geometry import finite,dot,cross,norm,unit
from target_pose import _solve
import rescue_config as cfg
from search_region import expand_to_frame
from boundary_policy import enabled as boundary_checks,visible_edges


def body_surface(shape_id,angles=24):
    height=cfg.OBJECT_HEIGHT_MM
    if shape_id not in (1,2,3):raise ValueError("INVALID_SHAPE_ID")
    points=[]
    levels=2 if shape_id!=3 else 9
    for level in range(levels):
        z=-height/2+height*level/(levels-1)
        if shape_id==1:radius=cfg.CYLINDER_DIAMETER_MM/2
        elif shape_id==2:
            ratio=(z+height/2)/height
            radius=(cfg.FRUSTUM_BOTTOM_DIAMETER_MM*(1-ratio)+cfg.FRUSTUM_TOP_DIAMETER_MM*ratio)/2
        else:
            R=cfg.DRUM_SIDE_ARC_RADIUS_MM
            radius=cfg.DRUM_END_DIAMETER_MM/2+math.sqrt(R*R-z*z)-math.sqrt(R*R-(height/2)**2)
        for n in range(angles):
            t=2*math.pi*n/angles
            points.append((radius*math.cos(t),radius*math.sin(t),z))
    return points


def _basis(theta,phi):
    axis=[math.sin(theta)*math.cos(phi),math.sin(theta)*math.sin(phi),math.cos(theta)]
    # Smooth fixed angular parameterization; no cross-product branch flips.
    first=[math.cos(theta)*math.cos(phi),math.cos(theta)*math.sin(phi),-math.sin(theta)]
    second=[-math.sin(phi),math.cos(phi),0.0]
    return axis,first,second


def project_surface(camera,parameters,surface):
    H=cfg.OBJECT_HEIGHT_MM
    C=[v*H for v in parameters[:3]]
    if not cfg.OBJECT_MIN_DEPTH_MM<=C[2]<=cfg.OBJECT_MAX_DEPTH_MM:raise ValueError("RESCUE_OBJECT_DEPTH_OUT_OF_RANGE")
    if not all(finite(v) for v in parameters):raise ValueError("NONFINITE_RESCUE_MODEL")
    axis,first,second=_basis(parameters[3],parameters[4]);out=[]
    for x,y,z in surface:
        px=C[0]+first[0]*x+second[0]*y+axis[0]*z
        py=C[1]+first[1]*x+second[1]*y+axis[1]*z
        pz=C[2]+first[2]*x+second[2]*y+axis[2]*z
        if pz<=10:raise ValueError("RESCUE_OBJECT_BEHIND_CAMERA")
        out.append((camera.fx*px/pz,camera.fy*py/pz))
    return out


def _supports(points,directions):
    return [max(x*dx+y*dy for x,y in points) for dx,dy in directions]


def _evaluate(camera,p,surface,directions,observed,jacobian=False):
    prediction=_supports(project_surface(camera,p,surface),directions)
    residual=[a-b for a,b in zip(prediction,observed)]
    if not jacobian:return residual,[]
    rows=[[0.0]*5 for _ in residual]
    for col in range(5):
        delta=0.002 if col<3 else 0.001
        shifted=list(p);shifted[col]+=delta
        values=_supports(project_surface(camera,shifted,surface),directions)
        for i,value in enumerate(values):rows[i][col]=(value-prediction[i])/delta
    return residual,rows


def _equations(residual,rows):
    H=[[sum(row[i]*row[j] for row in rows) for j in range(5)] for i in range(5)]
    g=[sum(row[i]*r for row,r in zip(rows,residual)) for i in range(5)]
    return H,g


def _refine(camera,initial,surface,directions,observed,iterations=15):
    p=list(initial);damping=0.01
    for _ in range(iterations):
        residual,rows=_evaluate(camera,p,surface,directions,observed,True)
        H,g=_equations(residual,rows);cost=sum(r*r for r in residual)
        improved=False
        for _ in range(5):
            A=[[H[i][j]+(damping*max(H[i][i],1e-6) if i==j else 0) for j in range(5)] for i in range(5)]
            try:
                step=_solve(A,[-v for v in g]);candidate=[a+b for a,b in zip(p,step)]
                after=sum(r*r for r in _evaluate(camera,candidate,surface,directions,observed)[0])
            except (ValueError,OverflowError,ZeroDivisionError):after=1e30
            if after<cost:
                p=candidate;damping=max(1e-7,damping/3);improved=True;break
            damping*=6
        if not improved or cost-after<1e-6 or sum(v*v for v in step)<1e-10:break
    residual,rows=_evaluate(camera,p,surface,directions,observed,True)
    return p,sum(r*r for r in residual),rows


class RescueObjectEstimator:
    def __init__(self,camera):self.camera=camera;self.previous={}
    def reset(self):self.previous={}
    def fit(self,pixels,shape_id,pixel_noise=0.8):
        try:
            if not 24<=len(pixels)<=256:raise ValueError("RESCUE_CONTOUR_POINT_COUNT")
            rays=[self.camera.ray(p) for p in pixels]
            points=[(self.camera.fx*r[0]/r[2],self.camera.fy*r[1]/r[2]) for r in rays]
            directions=[(math.cos(2*math.pi*i/24),math.sin(2*math.pi*i/24)) for i in range(24)]
            observed=_supports(points,directions)
            xmin,xmax=min(x for x,y in points),max(x for x,y in points)
            ymin,ymax=min(y for x,y in points),max(y for x,y in points)
            if min(xmax-xmin,ymax-ymin)<12:raise ValueError("RESCUE_OBJECT_TOO_SMALL")
            surface=body_surface(shape_id)
            mean=[sum(p[i] for p in points)/len(points) for i in range(2)]
            xx=sum((x-mean[0])**2 for x,y in points)
            yy=sum((y-mean[1])**2 for x,y in points)
            xy=sum((x-mean[0])*(y-mean[1]) for x,y in points)
            phi=0.5*math.atan2(2*xy,xx-yy)
            initials=[]
            if shape_id in self.previous:initials.append(self.previous[shape_id])
            else:
                for angle in (0.55,1.15,1.8,2.5):
                    for azimuth in (phi,phi+math.pi) if shape_id==2 else (phi,):
                        trial=[0.,0.,8.,angle,azimuth]
                        projected=project_surface(self.camera,trial,surface)
                        ratio=((max(x for x,y in projected)-min(x for x,y in projected))/(xmax-xmin)+(max(y for x,y in projected)-min(y for x,y in projected))/(ymax-ymin))/2
                        z=max(cfg.OBJECT_MIN_DEPTH_MM+40,min(cfg.OBJECT_MAX_DEPTH_MM-40,8*cfg.OBJECT_HEIGHT_MM*ratio))
                        initials.append([(xmin+xmax)/2/self.camera.fx*z/cfg.OBJECT_HEIGHT_MM,(ymin+ymax)/2/self.camera.fy*z/cfg.OBJECT_HEIGHT_MM,z/cfg.OBJECT_HEIGHT_MM,angle,azimuth])
            solutions=[]
            for initial in initials:
                try:solutions.append(_refine(self.camera,initial,surface,directions,observed))
                except (ValueError,OverflowError,ZeroDivisionError):pass
            if not solutions:raise ValueError("RESCUE_OBJECT_GEOMETRY_UNOBSERVABLE")
            solutions.sort(key=lambda v:v[1]);p,cost,rows=solutions[0]
            rms=math.sqrt(cost/len(observed))
            if boundary_checks() and rms>max(cfg.OBJECT_MAX_REPROJECTION_PX,2*pixel_noise):raise ValueError("RESCUE_OBJECT_REPROJECTION_ERROR")
            center=[v*cfg.OBJECT_HEIGHT_MM for v in p[:3]]
            for alternate,other_cost,_ in solutions[1:]:
                if other_cost<=cost+len(observed)*0.25 and norm([(a-b)*cfg.OBJECT_HEIGHT_MM for a,b in zip(p[:3],alternate[:3])])>cfg.OBJECT_AMBIGUITY_SEPARATION_MM:
                    raise ValueError("RESCUE_OBJECT_POSE_AMBIGUOUS")
            H,_=_equations([0.]*len(rows),rows)
            sigma=max(pixel_noise,rms,0.6)
            uncertainty=0.0
            for i in range(3):
                rhs=[float(i==j) for j in range(5)]
                diagonal=_solve(H,rhs)[i]
                if diagonal<=0 or not finite(diagonal):raise ValueError("RESCUE_OBJECT_GEOMETRY_UNOBSERVABLE")
                uncertainty=max(uncertainty,3*sigma*math.sqrt(diagonal)*cfg.OBJECT_HEIGHT_MM)
            if uncertainty>cfg.OBJECT_MAX_POSITION_3SIGMA_MM:raise ValueError("RESCUE_OBJECT_POSITION_UNCERTAIN")
            self.previous[shape_id]=list(p)
            return {"ok":True,"reason":"OK","position_camera_mm":center,
                    "reference_point":"rescue_object_axis_midpoint","orientation_valid":False,
                    "quality":{"reprojection_rms_px":rms,"position_3sigma_mm":uncertainty,
                               "boundary_checks_enabled":boundary_checks(),
                               "warnings":["RESCUE_OBJECT_REPROJECTION_ERROR"] if rms>max(cfg.OBJECT_MAX_REPROJECTION_PX,2*pixel_noise) else [],"model":"known_body_of_revolution_silhouette"}}
        except (ValueError,OverflowError,ZeroDivisionError) as exc:
            self.previous.pop(shape_id,None)
            return {"ok":False,"reason":str(exc),"position_camera_mm":None}


def extract_white_object(raw,bbox,_bounds=None):
    """Selected white component; search can expand throughout the full raw frame."""
    from target_color import _component
    x,y,w,h=bbox
    if not all(finite(v) for v in bbox) or min(w,h)<=0:raise ValueError("INVALID_RESCUE_ROI")
    if min(x,y)<0 or x+w>raw.width() or y+h>raw.height():raise ValueError("INVALID_RESCUE_ROI")
    if boundary_checks() and not cfg.OBJECT_FULL_FRAME_SEARCH and (min(x,y)<1 or x+w>=raw.width()-1 or y+h>=raw.height()-1):
        raise ValueError("RESCUE_TARGET_CLIPPED")
    margin=max(5,int(max(w,h)*0.12))
    if _bounds is None:
        x0,y0=max(0,int(x)-margin),max(0,int(y)-margin)
        x1,y1=min(raw.width(),int(x+w)+margin+1),min(raw.height(),int(y+h)+margin+1)
    else:
        x0,y0,x1,y1=_bounds
    step=1
    while ((x1-x0+step-1)//step)*((y1-y0+step-1)//step)>cfg.OBJECT_MAX_ROI_CELLS:step+=1
    columns,rows=(x1-x0+step-1)//step,(y1-y0+step-1)//step
    gray=raw.to_grayscale();values=bytearray(columns*rows);hist=[0]*256
    try:
        for gy in range(rows):
            for gx in range(columns):
                value=gray.get_pixel(x0+gx*step,y0+gy*step)
                values[gy*columns+gx]=value;hist[value]+=1
    finally:gray=None
    total=len(values);weighted=sum(i*n for i,n in enumerate(hist));weight=partial=0;best=-1;threshold=80
    for i in range(255):
        weight+=hist[i];partial+=i*hist[i]
        if not weight or weight==total:continue
        score=(weighted*weight-partial*total)**2/(weight*(total-weight))
        if score>best:best=score;threshold=i
    threshold=max(55,threshold)
    mask=bytearray(int(v>threshold) for v in values)
    components=[]
    for i in range(len(mask)):
        if mask[i]==1:
            item=_component(mask,i,columns,rows,1,2)
            inside=sum(x<=x0+(n%columns)*step<=x+w and y<=y0+(n//columns)*step<=y+h for n in item)
            if inside>=cfg.OBJECT_MIN_COMPONENT_SAMPLES:components.append((inside,item))
    if not components:raise ValueError("RESCUE_WHITE_OBJECT_NOT_FOUND")
    components.sort(key=lambda item:item[0],reverse=True);component=components[0][1]
    if len(components)>1 and components[1][0]>components[0][0]*0.35:raise ValueError("RESCUE_WHITE_COMPONENTS_AMBIGUOUS")
    if any(i%columns in (0,columns-1) or i//columns in (0,rows-1) for i in component):
        bounds=(x0,y0,x1,y1)
        expanded=expand_to_frame(bounds,raw.width(),raw.height())
        if cfg.OBJECT_FULL_FRAME_SEARCH and expanded!=bounds:
            return extract_white_object(raw,bbox,expanded)
        if boundary_checks():raise ValueError("RESCUE_TARGET_CLIPPED" if cfg.OBJECT_FULL_FRAME_SEARCH else "RESCUE_CONTOUR_BACKGROUND_CONTACT")
    occupied=set(component);boundary=[]
    for i in component:
        if any(n not in occupied for n,_,_ in visible_edges(i,columns,rows)):
            boundary.append((x0+(i%columns)*step,y0+(i//columns)*step))
    if len(boundary)>192:boundary=[boundary[i*len(boundary)//192] for i in range(192)]
    return boundary,max(0.8,step*0.6)
