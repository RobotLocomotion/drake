% ZMP Examples
%
% 2D Examples:
%   CoordinateFrames:
%     ZMP2D                  - x and z positions of the ZMP (z should be zero)
%    
%   System Models:
%     CartTable2D              - Input:  CartTable2DInput, 
%                                Output: CartTable2DState, CartTable2DFT
%     LinearInvertedPendulum2D - Input:  CartTable2DInput, 
%                                Output: [x,xdot of com], [x_zmp] with transforms to CartTable2DState, ZMP2D
%
%   Visualizers:
%     ZMP2DVisualizer          - Input: ZMP2D
%     FT2DVisualizer           - Input: CartTable2DState, CartTable2DFT
%     
%
%
% 3D Examples:
%   CartTable                - 
