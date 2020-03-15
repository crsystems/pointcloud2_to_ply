#include <atomic>
#include <cstring>

class AtomicOrientation {
	public:

		AtomicOrientation()
		{
			memset(quaternion, 0, 4*sizeof(double));
			memset(tmp_quaternion, 0, 4*sizeof(double));
		}

		~AtomicOrientation(){}

		void getOrientation(double *quat)
		{
			if(lock.test_and_set()){
				memcpy(quat, quaternion, 4*sizeof(double));
		
			} else {
				memcpy(quat, tmp_quaternion, 4*sizeof(double));
				memcpy(quaternion, tmp_quaternion, 4*sizeof(double));
				lock.clear();
			}
		}

		void setOrientation(double *quat)
		{
			while(lock.test_and_set()){}

			memcpy(tmp_quaternion, quat, 4*sizeof(double));

			lock.clear();
		}

	private:
		double quaternion[4];
		double tmp_quaternion[4];
		std::atomic_flag lock = ATOMIC_FLAG_INIT;
};
