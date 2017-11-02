clear all
close all
emg = load('shared/myo_emg.txt');
cmd = load('shared/cmd_p.txt');
tdist = load('shared/traj_dist');
nwp = load('shared/nwp.txt');
forces = load('shared/f.txt');
%angle = load('shared/angle.txt')    

%% Space distance on workspace
Distance = norm( cmd(:, 2:end) )
%% Align times
Times = [ emg(1,1) cmd(1,1) tdist(1,1) nwp(1,1) forces(1,1) ];
mt = min(Times);
emg(:,1) = emg(:,1) - mt;
round( emg(:,1), 2, 'significant' );
cmd(:,1) = cmd(:,1) - mt;
round( cmd(:,1), 2, 'significant' );
tdist(:,1) = tdist(:,1) - mt;
round( tdist(:,1), 2, 'significant' );
nwp(:,1) = nwp(:,1) - mt;
round( nwp(:,1), 2, 'significant' );
forces(:,1) = forces(:,1) - mt;
round( forces(:,1), 2, 'significant' );
%% wp times
wp_times = nwp(:,1);
%% Plot Fatigue
figure(1);
[h_emg, l_emg] = envelope( emg(:,9), 16, 'peak');
hold on
plot( emg(:,1), h_emg );
plot( emg(:,1), l_emg );
plot( emg(:,1), emg(:,9) );
legend('upper envelope', 'lower envelope', 'emg norm');
title('human fatigue');
