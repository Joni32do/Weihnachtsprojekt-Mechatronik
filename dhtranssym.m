function T = dhtranssym(varargin)
% dhtrans returns a homogeneus transformation matrix based on modified
% denavit-hartenberg convention as introduced in M8 in the course
% 'Modellierung und Simulation in der Mechatronik'
%
% a is a_{i-1}, the distance between K^{i-1} and K^i along x_{i-1}
% alp is alpha_{i-1}, the angle between z_{i-1} and z_i about x_{i-1}
% d is d_i, the distance between K^{i-1} and K^i along z_i
% theta is theta_i, the angle between x_{i-1} and x_i about z_{i-1}
%
% ckleinbach, 03.11.2014

a_ = [];
alp_ = [];
d_ = [];
theta_ = [];

for h_ = 2 : 2 : length(varargin)
    switch regexprep(lower(varargin{h_-1}),'[ _-]','')
        case 'id'
            id = varargin{h_};
        case 'a'
            a_ = varargin{h_};
            
        case 'alp'
            alp_ = varargin{h_};
            
        case 'd'
            d_ = varargin{h_};
            
        case 'theta'
            theta_ = varargin{h_};           
    end
end


syms a alp d theta
T = sym('T',[4 4]);

T(1,1) = cos(theta);
T(1,2) = -sin(theta);
T(1,3) = 0;
T(1,4) = a;
T(2,1) = cos(alp)*sin(theta);
T(2,2) = cos(alp)*cos(theta);
T(2,3) = -sin(alp);
T(2,4) = -sin(alp)*d;
T(3,1) = sin(alp)*sin(theta);
T(3,2) = sin(alp)*cos(theta);
T(3,3) = cos(alp);
T(3,4) = cos(alp)*d;
T(4,1) = 0;
T(4,2) = 0;
T(4,3) = 0;
T(4,4) = 1;

if ~isempty(a_)
    T = subs(T,a,a_);
end
if ~isempty(alp_)
    T = subs(T,alp,alp_);    
end
if ~isempty(d_)
    T = subs(T,d,d_);    
end
if ~isempty(theta_)
    T = subs(T,theta,theta_);    
end

T = subs(T,a,['a_',id-1]);
T = subs(T,alp,['alp_',id-1]);
T = subs(T,d,['d_',id]);
T = subs(T,theta,['theta_',id]);