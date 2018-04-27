function tilefigs(verticalFlag, figs)
%TILEFIGS Tile all open figure windows around on the screen.
%
%   TILEFIGS places all open figure windows around on the screen with no
%   overlap. TILEFIGS(ENFORCE_VERTICAL) distributes figures vertically first.
%   TILEFIGS(..., FIGS) can be used to specify which figures that
%   should be tiled. Figures are not sorted when specified.

%   Author:      Peter J. Acklam
%   Time-stamp:  2002-03-03 13:51:51 +0100
%   E-mail:      pjacklam@online.no
%   URL:         http://home.online.no/~pjacklam

%  Edited by Chris Wagner on 7/12/03 to not suck as much.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Get the handles to the figures to process.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if nargin < 2                              % If no input arguments...
   figs = findobj('Type', 'figure');    % ...find all figures.
   figs  = sort(figs);
end

if isempty(figs)
   disp('No open figures or no figures specified.');
   return
end

nfigs = length(figs);     % Number of figures.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Set miscellaneous parameter.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

nh = ceil(sqrt(nfigs));         % Number of figures horisontally.
nv = ceil(nfigs/nh);            % Number of figures vertically.


if nargin > 0
    if verticalFlag && nv<nh
        oldnv = nv;
        nv = nh;
        nh = oldnv;
    end
end


%nh = max(nh, 2);
%nv = max(nv, 2);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Get the screen size.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

set(0, 'Units', 'pixels');              % Set root units.
scrdim = get(0, 'ScreenSize');          % Get screen size.
scrwid = scrdim(3);                     % Screen width.
scrhgt = scrdim(4);                     % Screen height.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% The elements in the vector specifying the position.
% 1 - Window left position
% 2 - Window bottom position
% 3 - Window horizontal size
% 4 - Window vertical size
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

hspc   = 5;            % Horisontal space.
topspc = 80;            % Space above top figure.
medspc = 5;           % Space between figures.
botspc = 40;            % Space below bottom figure.

figwid = ( scrwid - (nh+1)*hspc )/nh;
fighgt = ( scrhgt - (topspc+botspc) - (nv-1)*medspc )/nv;

figwid = round(figwid);
fighgt = round(fighgt);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Put the figures where they belong.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for row = nv:-1:1
   for col = nh:-1:1
      idx = (row-1)*nh + col;
      if idx <= nfigs
         figlft = col*hspc + (col-1)*figwid;
         figbot = scrhgt - topspc - row*fighgt - (row-1)*medspc;
         figpos = [ figlft figbot figwid fighgt ];    % Figure position.
         fighnd = figs(idx);                          % Figure handle.
         units = get(fighnd, 'Units');                % Get units.
         set(fighnd, 'Units', 'pixels');              % Set new units.
         set(fighnd, 'Position', figpos);             % Set position.
         figure(fighnd);                              % Raise figure.
      end
   end
end
