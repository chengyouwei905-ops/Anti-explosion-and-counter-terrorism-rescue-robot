"""Expand a computation window within the full raw image, preserving coordinates."""


def expand_to_frame(bounds,width,height):
    x0,y0,x1,y1=bounds
    dx=max(8,(x1-x0+1)//2)
    dy=max(8,(y1-y0+1)//2)
    return (max(0,x0-dx),max(0,y0-dy),min(width,x1+dx),min(height,y1+dy))
