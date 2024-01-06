function [h] = christoffel(M, y, i, k, j)
% Christophel Symbole nach V-9-6/12  nach Vorlesung Modell. u. Sim. i.d. Mechatronik

h = 0.5 * ( diff(M(k,j),y(i)) + diff(M(k,i),y(j)) - diff(M(i,j),y(k)) );

end