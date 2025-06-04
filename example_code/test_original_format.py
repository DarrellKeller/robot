import csv

line = "200,200,200,200,200,0.2000,0.2000,0.2000,0.2000,0.2000,0.0000,0.0000,180,180,180,180"
# line = "L90,L45,F,R45,R90,sW_L90,sW_L45,sW_F,sW_R45,sW_R90,SteeringIn,PID_Out,L_Speed,R_Speed,BaseSpeed,CurSpeed"


values = list(csv.reader([line]))[0]
print(values) 