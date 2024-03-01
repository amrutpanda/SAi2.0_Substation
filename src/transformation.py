import numpy as np

T_a_b = np.array([[0.866,-0.5,0,11],[0.5,0.866,0,-1],[0,0,0,9],[0,0,0,1]])

p_a = np.array([[-1,-1,-1,1],[2,2,-2,-1],[4,2,3,7],[1,1,1,1]])

# p_b = np.array([[3,-4,2,1],[1,3,5,-3],[],[]])

p_b = T_a_b.dot(p_a)

print("p_a = \n",p_a)
print("p_b = \n",p_b)

T_a_b_ls = np.linalg.inv(p_a.dot(p_a.T)).dot(p_b.dot(p_a.T))

print("T_a_b_ls = \n",T_a_b_ls.T)
print("T_true = \n",T_a_b)

print("p_b_est = ", T_a_b_ls.dot(p_a))