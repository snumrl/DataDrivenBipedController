# +-------------------------------------------------------------------------
# | ysVpUtil.py
# |
# | Author: Yoonsang Lee
# +-------------------------------------------------------------------------
# | COPYRIGHT:
# |    Copyright Yoonsang Lee 2013
# |    See the included COPYRIGHT.txt file for further details.
# |    
# |    This file is part of the DataDrivenBipedController.
# |    DataDrivenBipedController is free software: you can redistribute it and/or modify
# |    it under the terms of the MIT License.
# |
# |    You should have received a copy of the MIT License
# |    along with DataDrivenBipedController.  If not, see <mit-license.org>.
# +-------------------------------------------------------------------------

def align2D(vpModel):
    lv = vpModel.getBodyVelocityGlobal(0)
    vpModel.setBodyVelocityGlobal(0, (lv[0], lv[1], 0.))
    av = vpModel.getBodyAngVelocityGlobal(0)
    vpModel.setBodyAngVelocityGlobal(0, (av[0], 0., av[2]))
