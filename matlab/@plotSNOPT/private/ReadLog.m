%PlotSNOPT/ReadSNOPTLog PlotSNOPT class log reading function.
%   s = ReadSNOPTLog('logpath') extracts key parameters from the SNOPT output log
%   file specified at logpath.
%
%   silvaw 04-30-15
%   shawcortez 10-08-13

function [ Out ] = ReadSNOPTLog(self,filepath)
%*=+--+=#=+--      EA-DDDAS Trajectory Optimization Layer         --+=#=+--+=#*%
%          Copyright (C) 2015 Regents of the University of Colorado.           %
%                             All Rights Reserved.                             %
%                                                                              %
%    This program is free software: you can redistribute it and/or modify      %
%    it under the terms of the GNU General Public License version 2 as         %
%    published by the Free Software Foundation.                                %
%                                                                              %
%    This program is distributed in the hope that it will be useful,           %
%    but WITHOUT ANY WARRANTY; without even the implied warranty of            %
%    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the             %
%    GNU General Public License for more details.                              %
%                                                                              %
%    You should have received a copy of the GNU General Public License         %
%    along with this program.  If not, see <http://www.gnu.org/licenses/>.     %
%                                                                              %
%           Will Silva                                                         %
%           william.silva@colorado.edu                                         %
%                                                                              %
%*=+--+=#=+--                 --+=#=+--+=#=+--                    --+=#=+--+=#*%

%open log
fid = fopen(filepath);

%---------------------
% Read Section 1
fgets(fid);
tline = fgets(fid);
end_sec = 0;
ii = 1;
while ischar(tline) && (end_sec == 0)
    
    A = sscanf(tline,'%s');
    
    if length(A) > 10
        if strcmp(A(1:10),'SNMEMAEXIT')
            SNMEMA_EXIT = A(11:end);
        end
        if strcmp(A(1:10),'SNMEMAINFO')
            SNMEMA_INFO = A(11:end);
        end
    end
    
    if length(A) == 1
        if str2num(A) == 1
            end_sec = 1;
        end
    end
    
    tline = fgets(fid);
    ii = ii+1;
end

%---------------------
% Read Section 2
end_sec = 0;
while ischar(tline) && (end_sec == 0)
    
    A = sscanf(tline,'%s');
    
    if length(A) == 1
        if str2num(A) == 1
            end_sec = 1;
        end
    end
    
    tline = fgets(fid);
    ii = ii+1;
end

%---------------------
% Read Section 3
end_sec = 0;
while ischar(tline) && (end_sec == 0)
    
    A = sscanf(tline,'%s');
    
    if length(A) == 1
        if str2num(A) == 1
            end_sec = 1;
        end
    end
    
    tline = fgets(fid);
    ii = ii+1;
end

%---------------------
% Read Section 4
end_sec = 0;
while ischar(tline) && (end_sec == 0)
    
    A = sscanf(tline,'%s');
    
    if length(A) > 10
        if strcmp(A(1:24),'-->Thelargestdiscrepancy')
            CONSTGRAD_CHECK_s = A(28:35);
            CONSTGRAD_CHECK = str2num(CONSTGRAD_CHECK_s);
        end
        if strcmp(A(1:17),'Gradientprojected')
            OBJPROJ_CHECK_s = A(32:end);
            OBJPROJ_CHECK = str2num(OBJPROJ_CHECK_s);
        end
        if strcmp(A(1:23),'Differenceapproximation')
            OBJDIFF_CHECK_s = A(24:end);
            OBJDIFF_CHECK = str2num(OBJDIFF_CHECK_s);
        end
    end
    
    if length(A) == 1
        if str2num(A) == 1
            end_sec = 1;
        end
    end
    
    tline = fgets(fid);
    ii = ii+1;
end

%---------------------
% Read Section 5
end_sec = 0;
while ischar(tline) && (end_sec == 0)
    
    A = sscanf(tline,'%s');
    
    if length(A) == 1
        if str2num(A) == 1
            end_sec = 1;
        end
    end
    
    tline = fgets(fid);
    ii = ii+1;
end

%---------------------
% Read Section 6
end_sec = 0;
while ischar(tline) && (end_sec == 0)
    
    A = sscanf(tline,'%s');
    
    if length(A) > 10
        if strcmp(A(1:10),'SNOPTAEXIT')
            SNOPTA_EXIT = A(11:end);
        end
        if strcmp(A(1:10),'SNOPTAINFO')
            SNOPTA_INFO = A(11:end);
        end
    end
    
    if length(A) > 42
        if strcmp(A(1:15),'No.ofiterations')
            ITER_NUM_s = A(16:19);
            ITER_NUM = str2num(ITER_NUM_s);
        end
    end
    
    if length(A) == 1
        if str2num(A) == 1
            end_sec = 1;
        end
    end
    
    tline = fgets(fid);
    ii = ii+1;
end

%---------------------
% Read Section 7
while ischar(tline) 
    
    A = sscanf(tline,'%s');
    
    if length(A) > 20
        if strcmp(A(1:21),'Timeforsolvingproblem')
            SNOPT_TIME = A(22:end);
        end
    end
    
    tline = fgets(fid);
    ii = ii+1;
end


fclose(fid);

Out.SNMEMA_EXIT     = SNMEMA_EXIT;
Out.SNMEMA_INFO     = SNMEMA_INFO;
Out.CONSTGRAD_CHECK = CONSTGRAD_CHECK;
Out.OBJPROJ_CHECK   = OBJPROJ_CHECK;
Out.OBJDIFF_CHECK   = OBJDIFF_CHECK;
Out.SNOPTA_EXIT     = SNOPTA_EXIT;
Out.SNOPTA_INFO     = SNOPTA_INFO;
Out.ITER_NUM        = ITER_NUM;
Out.SNOPT_TIME      = SNOPT_TIME;

end

