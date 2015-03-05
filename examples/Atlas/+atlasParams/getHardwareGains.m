function gains = getHardwareGains(r)

nu = r.getNumInputs();

if isempty(r.atlas_version)
  atlas_version = 3;
else
  atlas_version = r.atlas_version;
end

gains = struct();
gains.k_f_p    = zeros(nu,1);
gains.k_q_p    = zeros(nu,1);
gains.k_q_i    = zeros(nu,1);
gains.k_qd_p   = zeros(nu,1);
gains.ff_qd    = zeros(nu,1);
gains.ff_f_d   = zeros(nu,1);
gains.ff_const = zeros(nu,1);
gains.ff_qd_d  = zeros(nu,1);