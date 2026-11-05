"""Optional contour acceptance policy; memory and numerical checks are separate."""
import config


def enabled():
    return config.BOUNDARY_CHECKS_ENABLED


def visible_edges(index,columns,rows):
    """In-grid neighbours only: the image cut itself is not an observed contour."""
    x,y=index%columns,index//columns
    if x:yield index-1,-0.5,0.0
    if x+1<columns:yield index+1,0.5,0.0
    if y:yield index-columns,0.0,-0.5
    if y+1<rows:yield index+columns,0.0,0.5
