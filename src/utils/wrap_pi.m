function angle = wrap_pi(angle)
%WRAP_PI Wrap angle values to [-pi, pi).

angle = mod(angle + pi, 2*pi) - pi;
end
