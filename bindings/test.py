from test import test

test()

print("test passed")



from opt_jump import OptJump 
import numpy as np

op = OptJump()
x_0 = np.array([0.0, 0.0, 0.5])
e_0 = np.zeros(3)
E = []
for x in [-0.25,0.25]:
    for y in [-0.25,0.25]:
        E.append(np.array([x,y,0.0]))



op.set_initial_base_state(x_0, e_0)
op.set_jump_length(2.0)
op.set_initial_EE_state(E)

op.solve()