#include <iostream>
#include <iomanip>
#include <string>
#include <exception>
#include <tiledb/tiledb>
#include <tiledb/array.h>

int main(int argc, char **argv)
{
    try
    {
        // This is ridiculous
        tiledb::Config config;
        config["vfs.gcs.project_id"] = "classifysentimen-1575308420153";
        tiledb::Context ctx;

        // Create the dimension: Values 1 through 4 possible, tile extent 2
        auto dim = tiledb::Dimension::create<int32_t>(ctx, "dim", {{1, 4}}, 2);
        tiledb::Domain dom(ctx);
        dom.add_dimension(dim);
        auto attr = tiledb::Attribute::create<int32_t>(ctx, "attr");

        // Add domain (with dimensions) and attributes to a schema
        tiledb::ArraySchema schema(ctx, TILEDB_DENSE);
        schema.set_domain(dom);
        schema.add_attribute(attr);

        // Specify the URI to the GCS bucket
        std::string bucket = "classifysentimen-1575308420153.appspot.com";
        std::string uri = "gcs://" + bucket + "/test1";

        // Create the array in the GCS bucket
        //tiledb::Array::create(uri, schema);
        std::vector<int32_t> data = {0, 1, 2, 3};
        std::vector<int> sub = {1, 4};
        tiledb::Array array(ctx, uri, TILEDB_WRITE);
        tiledb::Query query(ctx, array);
        query.set_layout(TILEDB_ROW_MAJOR)
            .set_buffer("attr", data)
            .set_subarray(sub);
        query.submit();
        array.close();

        // Read the array back from GCS bucket
        std::vector<int32_t> outp(5);
        tiledb::Array brray(ctx, uri, TILEDB_READ);
        tiledb::Query ruery(ctx, brray);
        ruery.set_layout(TILEDB_ROW_MAJOR)
            .set_buffer("attr", outp)
            .set_subarray(sub);
        ruery.submit();
        for (auto value: outp)
            std::cout << std::setw(5) << value;
        std::cout << std::endl;
        brray.close();

        // Load array from disk
        tiledb::Context ctx2;
        std::string uri2 = "./array_tiledb";
        std::cout << "Loading the local array schema... " << std::flush;
        tiledb::ArraySchema schema2(ctx2, uri2);
        std::cout << "Done." << std::endl << std::flush;
        std::cout << schema2 << std::endl;
    }
    catch (const std::exception& e)
    {
        std::cout << e.what() << std::endl;
    }
    return 0;
}