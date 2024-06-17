clear;
clc;
z=1;
%Mass of Arm
Arm = 0.05;
LeftLeg = linspace(0,0.15,100);
RightLeg = linspace(0,0.15,100);
F_LeftLeg = 0.5*1.2*1000*0.05*0.05*pi.*LeftLeg*0.04;
F_RightLeg = 0.5*1.2*1000*0.05*0.05*pi.*RightLeg*0.04;

%Angular Acceleration vs Arm Length
for i = 1:100
    TorqueLeft(1,i)  = F_LeftLeg(1,i)*((LeftLeg(1, i)/2)+0.09);
    TorqueRight(1,i) = F_RightLeg(1,i)*-((LeftLeg(1, i)/2)+0.09);
end

for i = 1:100
    for j = 1:100
        NetTorque(1,z) = TorqueLeft(1,i) + TorqueRight(1,j);
        LeftLegLength(1,z) = LeftLeg(1,i);
        RightLegLength(1,z) = RightLeg(1,j);
        LeftMass = Arm*(LeftLeg(1,i)/0.15);
        RightMass = Arm*(RightLeg(1,j)/0.15);
        BodyMass = 0.25 - LeftMass - RightMass;
        MomentLeft(1,z) = 0.25*LeftMass*0.04+(1/12)*LeftMass*(LeftLegLength(1,z)^2);
        MomentRight(1,z) = 0.25*RightMass*0.04+(1/12)*RightMass*(RightLegLength(1,z)^2);
        NetAngAccel(1,z) = (TorqueLeft(1,i)/MomentLeft(1,z)) + (TorqueRight(1,j)/MomentRight(1,z));
        F_body = 0.5*0.6*1000*0.05*0.05*pi*0.09*0.05;
        LinearAcceleration(1,z) = F_body/BodyMass;
        z = z+1;
    end
end
MaxTorque = max(NetTorque);
%plot3(LeftLegLength,RightLegLength, NetTorque);
scatter3(LeftLegLength, RightLegLength, NetAngAccel);
xlabel("Left Leg Length (m)")
ylabel("Right Leg Length (m)")
zlabel("Angular Acceleration (Radians/Sec^2)")
title("Angular Acceleration vs Leg Length")

LinearAcceleration = fliplr(LinearAcceleration);
figure 
scatter3(LeftLegLength, RightLegLength, LinearAcceleration);
xlabel("Left Leg Length (m)")
ylabel("Right Leg Length (m)")
zlabel("Linear Acceleration (Radians/Sec^2)")
title("Linear Acceleration vs Leg Length")
