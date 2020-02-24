/*
  Copyright <2018-2019> <Ainstein, Inc.>

  Redistribution and use in source and binary forms, with or without modification, are permitted
  provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice, this list of
  conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright notice, this list of
  conditions and the following disclaimer in the documentation and/or other materials provided
  with the distribution.

  3. Neither the name of the copyright holder nor the names of its contributors may be used to
  endorse or promote products derived from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
  IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
  FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
  IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
  OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "ainstein_radar_filters/tracking_filter.h"

namespace ainstein_radar_filters
{
const int TrackingFilter::max_tracked_targets = 100;

void TrackingFilter::initialize(void)
{
  // Reserve space for the maximum number of target Kalman Filters:
  filters_.reserve(TrackingFilter::max_tracked_targets);

  // Launch the periodic filter update thread:
  filter_process_thread_ =
	  std::unique_ptr<std::thread>(new std::thread(&TrackingFilter::processFiltersLoop, this, filter_process_rate_));
}

void TrackingFilter::processFiltersLoop(double frequency)
{
  // Enter the main filters update loop:
  std::chrono::system_clock::time_point time_now, time_prev;
  double dt;
  double thread_period = (1.0 / frequency);
  bool first_time = true;
  while (is_running_)
  {
	// Compute the actual delta time:
	if (first_time)
	{
	  time_prev = std::chrono::system_clock::now();
	  first_time = false;
	}

	time_now = std::chrono::system_clock::now();

	dt = (time_now - time_prev).count();

	// Block callback from modifying the filters
	mutex_.lock();

	// Remove filters which have not been updated in specified time:
	if (print_debug_)
	{
	  std::cout << "Number of filters before pruning: " << filters_.size() << std::endl;
	}
	if (filters_.size() > 0)
	{
	  filters_.erase(
		  std::remove_if(filters_.begin(), filters_.end(),
						 [&](const RadarTargetKF& kf) { return (kf.getTimeSinceUpdate() > filter_timeout_); }),
		  filters_.end());
	}
	if (print_debug_)
	{
	  std::cout << "Number of filters after pruning: " << filters_.size() << std::endl;
	}

	// Run process model for each filter:
	for (auto& kf : filters_)
	{
	  kf.process(dt);
	}

	// Update which filters are currently tracking (based on time alive):
	for (int i = 0; i < filters_.size(); ++i)
	{
	  if (filters_.at(i).getTimeSinceStart() >= filter_min_time_)
	  {
		is_tracked_.at(i) = true;
	  }
	  else
	  {
		is_tracked_.at(i) = false;
	  }
	}

	// Release lock on filter state
	mutex_.unlock();

	// Store the current time and velocity:
	time_prev = time_now;

	// Sleep to maintain desired freq:
	double sleep_time = thread_period - (std::chrono::system_clock::now() - time_now).count();
	if (sleep_time > 1e-9)
	{
	  std::this_thread::sleep_for(std::chrono::duration<double>(sleep_time));
	}
	else
	{
	  std::cout << "Warning >> process thread period overrun." << std::endl;
	}
  }
}

void TrackingFilter::updateFilters(const std::vector<RadarTarget>& targets)
{
  // Reset the measurement count vector for keeping track of which measurements get used:
  meas_count_vec_.resize(targets.size());
  std::fill(meas_count_vec_.begin(), meas_count_vec_.end(), 0);

  // Block update loop from modifying the filters
  mutex_.lock();

  // Resize the targets associated with each filter:
  filter_targets_.clear();
  filter_targets_.resize(filters_.size());

  // Pass the raw detections to the filters for updating:
  for (int i = 0; i < filters_.size(); ++i)
  {
	if (print_debug_)
	{
	  std::cout << filters_.at(i) << std::endl;
	}
	for (int j = 0; j < targets.size(); ++j)
	{
	  // Only use this target if it hasn't already been used by a filter:
	  if (meas_count_vec_.at(j) == 0)
	  {
		// Check whether the target should be used as measurement by this filter:
		RadarTarget t = targets.at(j);
		Eigen::Vector4d z = filters_.at(i).computePredMeas(filters_.at(i).getState());
		Eigen::Vector4d y = Eigen::Vector4d(t.range, t.speed, t.azimuth, t.elevation);

		// Compute the normalized measurement error (squared):
		double meas_err =
			(y - z).transpose() * filters_.at(i).computeMeasCov(filters_.at(i).getState()).inverse() * (y - z);

		if (print_debug_)
		{
		  std::cout << "Meas Cov Inv: " << filters_.at(i).computeMeasCov(filters_.at(i).getState()).inverse()
					<< std::endl;
		  std::cout << "Target " << j << " meas_err: " << meas_err << std::endl;
		  std::cout << "Target " << j << ": " << std::endl
					<< t.range << " " << t.speed << " " << t.azimuth << " " << t.elevation << std::endl;
		}

		// Allow the measurement through the validation gate based on threshold:
		if (meas_err < filter_val_gate_thresh_)
		{
		  filters_.at(i).update(t.range, t.speed, t.azimuth, t.elevation);
		  ++meas_count_vec_.at(j);

		  // Store the target associated with the filter:
		  filter_targets_.at(i).push_back(t);
		}
	  }
	}
  }

  // Iterate through targets and push back new KFs for unused measurements:
  std::vector<RadarTarget> arr;
  for (int i = 0; i < meas_count_vec_.size(); ++i)
  {
	if (meas_count_vec_.at(i) == 0)
	{
	  if (print_debug_)
	  {
		std::cout << "Pushing back new filter: " << targets.at(i).range << " " << targets.at(i).speed << " "
				  << targets.at(i).azimuth << " " << targets.at(i).elevation << std::endl;
	  }
	  filters_.emplace_back(targets.at(i).range, targets.at(i).speed, targets.at(i).azimuth, targets.at(i).elevation);

	  // Make sure to push back an empty array of targets associated with the new filter
	  filter_targets_.push_back(arr);
	}
  }

  // Release lock on filter state
  mutex_.unlock();
}

void TrackingFilter::getTrackedObjects(std::vector<RadarTarget>& tracked_objects)
{
  tracked_objects.clear();
  for (int i = 0; i < filters_.size(); ++i)
  {
	if (is_tracked_.at(i))
	{
	  RadarTargetKF::FilterState state = filters_.at(i).getState();
	  RadarTarget object(state.range, state.speed, state.azimuth, state.elevation);
	  tracked_objects.push_back(object);
	}
  }
}

void TrackingFilter::getTrackedObjectTargets(std::vector<std::vector<RadarTarget>>& tracked_object_targets)
{
  tracked_object_targets.clear();
  for (int i = 0; i < filters_.size(); ++i)
  {
	if (is_tracked_.at(i))
	{
	  tracked_object_targets.push_back(filter_targets_.at(i));
	}
  }
}

}  // namespace ainstein_radar_filters