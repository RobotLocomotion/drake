function [utraj,xtraj,info] = dircol(sys,varargin)

[utraj,xtraj,info] = trajectoryOptimization(sys,@dircol_snopt_transcription,varargin{:});



