"""LAB threshold helpers. CanMV segmentation uses native RGB565 Image.binary."""


def matches_lab(lab,threshold):
    return (threshold[0]<=lab[0]<=threshold[1] and
            threshold[2]<=lab[1]<=threshold[3] and
            threshold[4]<=lab[2]<=threshold[5])


def rgb_to_lab_reference(rgb):
    """sRGB/D65 reference for host images lacking CanMV's native binary API."""
    values=[]
    for value in rgb:
        v=value/255.0
        values.append(v/12.92 if v<=.04045 else ((v+.055)/1.055)**2.4)
    r,g,b=values
    xyz=((.4124564*r+.3575761*g+.1804375*b)/.95047,
         .2126729*r+.7151522*g+.0721750*b,
         (.0193339*r+.1191920*g+.9503041*b)/1.08883)
    f=[v**(1.0/3.0) if v>.008856451679 else 7.787037037*v+16.0/116.0 for v in xyz]
    return (116.0*f[1]-16.0,500.0*(f[0]-f[1]),200.0*(f[1]-f[2]))


def native_lab_mask(rgb_image,threshold):
    """Preserve calibrated geometry and source pixels; None selects host fallback."""
    if not hasattr(rgb_image,"copy") or not hasattr(rgb_image,"binary"):return None
    mask=rgb_image.copy()
    mask.binary([tuple(threshold)])
    return mask
