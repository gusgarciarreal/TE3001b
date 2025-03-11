# Importing Libraries
import roboticstoolbox as rtb
panda = rtb.models.ETS.Panda()
puma = rtb.models.ETS.Puma560()
#Puma
f_kinematics_puma = puma.fkine(puma.qr)
i_kinematics_puma = puma.ikine_LM(f_kinematics_puma)
print("Default angular position qr")
print(puma.qr)
print("Inverse Kinematics")
print(i_kinematics_puma)
#Panda
f_kinematics = panda.fkine(panda.qr)
i_kinematics = panda.ikine_LM(f_kinematics)
print("Default angular position qr")
print(panda.qr)
print("Inverse Kinematics")
print(i_kinematics)
