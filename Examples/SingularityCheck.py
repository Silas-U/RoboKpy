"""
Author: Silas Udofia
Date: 2024-08-02
GitHub: https://github.com/Silas-U/RoboKpy/tree/main

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at: http://www.apache.org/licenses/LICENSE-2.0
"""

from robokpy import Init_Model
from Models.Model import DHModel

model = DHModel.get_model('Puma561')
rb = Init_Model(model, robot_name='Puma561')

conf = [90, 45, -60, 20, 0, 90]
rb.fk.compute(conf)
rb.jac.check_singularity()
