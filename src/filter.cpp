
double F_ext = 	0;           // filtered force value
double F_bias = 0;          // steady state component
double F_new = 	0;           // new force measured
double alpha =  0.1;    // dissipative filter parameter
double beta =   0.001;  // low-pass filter parameter

// LOW PASS FILTER
F_bias = F_bias + beta*(F_new);

// DISSIPATIVE FILTER
double F_new_ = F_new - F_bias;
double F_ext_ = F_ext - F_bias;
if (F_new_*F_ext_ > 0){
	if (std::abs(F_new_) > std::abs(F_ext_)){
		F_ext = F_ext + alpha * (F_new - F_ext);
	}else{
		F_ext = F_new;
	}
}else{
	F_ext = F_bias + alpha * (F_new_);
}