function [cm] = fcn_cm(q)

cm = zeros(3,1);

  cm(1,1)=(1312114744235139161211*cos(q(1) + q(2) + q(3)))/35926340127441289216000 + (45*cos(q(1) +...
          q(2) + q(3) + q(4) - q(5) - q(6)))/63818 + (33755379927067341335*cos(q(1) + q(2) + q(3) +...
          q(4) - q(5)))/18394286145249940078592 + (45*cos(q(1) + q(2) + q(3) + q(4) + q(5) + q(6)))/63818 + (9500534366562261861267*cos(q(1) + q(2) +...
          q(3) + q(4)))/7357714458099976031436800 + (968237891087637672277*cos(q(1) +...
          q(2)))/22992857681562425098240 + (2240226516346356329569*cos(q(1)))/54100841603676294348800 + (33755379927067341335*cos(q(1) +...
          q(2) + q(3) + q(4) + q(5)))/18394286145249940078592;
  cm(2,1)=(45*sin(q(5) + q(6)))/31909 + (33755379927067341335*sin(q(5)))/9197143072624970039296;
  cm(3,1)=- (33755379927067341335*sin(q(1) + q(2) + q(3) + q(4) +...
          q(5)))/18394286145249940078592 - (1312114744235139161211*sin(q(1) + q(2) + q(3)))/35926340127441289216000 - (45*sin(q(1) + q(2) + q(3) +...
          q(4) - q(5) - q(6)))/63818 - (33755379927067341335*sin(q(1) + q(2) + q(3) + q(4) - q(5)))/18394286145249940078592 - (45*sin(q(1) + q(2) + q(3) + q(4) +...
          q(5) + q(6)))/63818 - (9500534366562261861267*sin(q(1) + q(2) + q(3) +...
          q(4)))/7357714458099976031436800 - (968237891087637672277*sin(q(1) + q(2)))/22992857681562425098240 - (2240226516346356329569*sin(q(1)))/54100841603676294348800;

 