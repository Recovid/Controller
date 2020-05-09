




// static float A_calibrated;
// static float B_calibrated;

// #define MAX_FLOW_SAMPLES	        400
// #define FLOW_SAMPLING_PERIOD_MS   5
// static float 			_flow_samples[MAX_FLOW_SAMPLES];		//  used as buffer for flow analysis and motor command computing
// static uint32_t 	_flow_samples_count;



// static float compte_motor_step_time(uint32_t step_number, float desired_flow_Ls, float A, float B, float speed);
// static void adaptation(float target_flow_Lm, float* flow_samples, uint32_t nb_samples, float time_step_sec, float* A, float* B);
// static float linear_fit(float* samples, uint32_t samples_len, float time_step_sec, float* slope);
// static int32_t get_plateau(float* samples, uint32_t samples_len, float time_step_sec, uint8_t windows_number, uint32_t* low_bound, uint32_t* high_bound);

// static float compte_motor_step_time(uint32_t step_number, float desired_flow_Ls, float A, float B, float speed) {
// 	float res = (0.8*A*speed*speed*step_number) + B * speed;
// 	res = res / desired_flow_Ls;
// 	if (res * 1000000 < 200) {return 200;}
// 	else {return res * 1000000.;}
// }


// static void adaptation(float target_flow_Lm, float* flow_samples, uint32_t nb_samples, float time_step_sec, float* A, float* B) {
//   if(nb_samples==0) return;
// //************************************************* PID ZONE ********************************************//
// 		// Compute average flow and slope to adjust A_calibrated and B_calibrated
// 		float P_plateau_slope = 0.1;
// 		float P_plateau_mean = 0.2;
// 		uint32_t low;
// 		uint32_t high;
// 		if(get_plateau(flow_samples, nb_samples, time_step_sec, 10, &low, &high) == 0) {
// //			brth_printf("plateau found from sample %lu to %lu\n", low, high);
// 		} else {
// //			brth_printf("plateau NOT found, considering from sample %lu to %lu\n", low, high);
// 		}
// 		float plateau_slope = linear_fit(flow_samples+low, high-low-1, time_step_sec, &plateau_slope);
// 		float plateau_mean = 0;
// 		for(uint32_t i=low; i<high; i++) {
// 			plateau_mean += flow_samples[i];
// 		}
// 		plateau_mean = plateau_mean/(high-low);
// //		brth_printf("plateau slope : %ld\n",(int32_t)(1000*plateau_slope));
// //		brth_printf("plateau mean : %ld\n",(int32_t)(1000*plateau_mean));

// 		float error_mean = plateau_mean - (target_flow_Lm/60.);

// 		*A += plateau_slope * P_plateau_slope;
// 		*B += error_mean * P_plateau_mean;
// //		brth_printf("A = %ld\n", (int32_t)(1000*(*A)));
// //		brth_printf("B = %ld\n", (int32_t)(1000*(*B)));

// }

// // Compute slope of samples fetched with specified time_step
// // Returns 	R  if fit is ok
// // 			-1 if fit is not possible
// static float linear_fit(float* samples, uint32_t samples_len, float time_step_sec, float* slope){
// 	float sumx=0,sumy=0,sumxy=0,sumx2=0, sumy2=0;
// 	for(uint32_t i=0;i<samples_len;i++) {
// 		sumx  = sumx + (float)i * time_step_sec;
// 		sumx2 = sumx2 + (float)i*time_step_sec*(float)i*time_step_sec;
// 		sumy  = sumy + *(samples+i);
// 		sumy2 = sumy2 + (*(samples+i)) * (*(samples+i));
// 		sumxy = sumxy + (float)i*(time_step_sec)* (*(samples+i));
// 	}
// 	float denom = (samples_len * sumx2 - (sumx * sumx));
// 	if(denom == 0.) {
// //		brth_printf("Calibration of A is not possible\n");
// 		return 1;
// 	}
// 	// compute slope a
// 	*slope = (samples_len * sumxy  -  sumx * sumy) / denom;

// 	// compute correlation coefficient
// 	return (sumxy - sumx * sumy / samples_len) / sqrtf((sumx2 - (sumx*sumx)/samples_len) * (sumy2 - (sumy*sumy)/samples_len));
// }

// static int32_t get_plateau(float* samples, uint32_t samples_len, float time_step_sec, uint8_t windows_number, uint32_t* low_bound, uint32_t* high_bound){
// 	if(windows_number < 2 || windows_number > 30) {return -1;}
// 	float slopes[30];
// 	*high_bound = samples_len-1;
// 	// Compute slope for time windows to detect when signal start increasing/decreasing
// 	for(uint32_t window=0; window<windows_number; window++) {
// 		float r = linear_fit(samples+window*(samples_len/windows_number), samples_len/windows_number, time_step_sec, slopes+window);
// //		brth_printf("%ld    ", (int32_t)(*(slopes+window) * 1000));
// 	}
// //	brth_printf("\n");
// 	for(uint32_t window=1; window<windows_number; window++) {
// 		float delta_slope = slopes[window-1] - slopes[window];
// 		if(delta_slope > 1.) {
// 			*low_bound = (uint32_t)((samples_len/windows_number)*(window+1));
// //			brth_printf("plateau begin at %lu over %lu points\n", *low_bound, (uint32_t)samples_len);
// 			return 0;
// 		}
// 	}
// 	*low_bound = (uint32_t)(samples_len/2);
// //	brth_printf("No plateau found\n");
// 	return 1;
// }
