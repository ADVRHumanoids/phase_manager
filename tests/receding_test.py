from horizon.problem import Problem
import phase_manager.pymanager as pymanager
import phase_manager.pyphase as pyphase

import numpy as np

ns = 20
dt = 0.01
prb = Problem(ns, receding=True)
prb.setDt(dt)

a = prb.createStateVariable('a', 1)
b = prb.createStateVariable('b', 1)

cnsrt1 = prb.createConstraint('cnsrt1', a, nodes=[])
cnsrt2 = prb.createConstraint('cnsrt2', 2 * b, nodes=[])
# cnsrt3 = prb.createConstraint('cnsrt3', 3 * a, nodes=[])

pm = pymanager.PhaseManager(ns)
timeline_1 = pm.addTimeline('timeline_1')
phase_1 = pyphase.Phase(5, 'phase_1')

phase_1.addItem(cnsrt1)
phase_1.addItem(cnsrt2)
phase_1.addItem(cnsrt1)

# phase_1.addItem(cnsrt3)

timeline_1.registerPhase(phase_1)

timeline_1.addPhase(phase_1)
timeline_1.addPhase(phase_1)
timeline_1.addPhase(phase_1)

print(" initial condition: ")

print("cnsrt1: ", cnsrt1.getNodes())
print("cnsrt2: ", cnsrt2.getNodes())
# print("cnsrt3: ", cnsrt3.getNodes())

pm._shift_phases()
print(" SHIFT PHASES: ")

print("cnsrt1: ", cnsrt1.getNodes())
print("cnsrt2: ", cnsrt2.getNodes())
# print("cnsrt3: ", cnsrt3.getNodes())

pm._shift_phases()
print(" SHIFT PHASES: ")

print("cnsrt1: ", cnsrt1.getNodes())
print("cnsrt2: ", cnsrt2.getNodes())
# print("cnsrt3: ", cnsrt3.getNodes())

exit()
