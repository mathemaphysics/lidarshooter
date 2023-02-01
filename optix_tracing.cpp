extern "C" __global__ void __raygen__from_buffer()
{
	const uint3        idx        = optixGetLaunchIndex();
	const uint3        dim        = optixGetLaunchDimensions();
	const unsigned int linear_idx = idx.z * dim.y * dim.x + idx.y * dim.x + idx.x;

	unsigned int t, nx, ny, nz;
	Ray          ray = params.rays[linear_idx];
	optixTrace( params.handle, ray.origin, ray.dir, ray.tmin, ray.tmax, 0.0f, OptixVisibilityMask( 1 ),
			OPTIX_RAY_FLAG_NONE, RAY_TYPE_RADIANCE, RAY_TYPE_COUNT, RAY_TYPE_RADIANCE, t, nx, ny, nz );

	Hit hit;
	hit.t                   = int_as_float( t );
	hit.geom_normal.x       = int_as_float( nx );
	hit.geom_normal.y       = int_as_float( ny );
	hit.geom_normal.z       = int_as_float( nz );
	params.hits[linear_idx] = hit;
}
