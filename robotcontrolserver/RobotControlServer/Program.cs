
using Microsoft.OpenApi.Models;
using System.Reflection.Metadata;
using NLog.Web;
using MQTTnet.AspNetCore;
using Microsoft.AspNetCore.Hosting;
using Microsoft.Extensions.Hosting;
using MQTTnet.AspNetCore.Routing;
using System.Runtime.InteropServices;
using ConfigApp;
using RobotServer.Startups;

namespace RobotServer
{
    public class Program
    {
        public static void Main(string[] args)
        {
            string ConfigFolder = "ConfigApp";
            string ConfigLogFolder = "ConfigLog/nlog.config";
            #if RELEASE
            if (RuntimeInformation.IsOSPlatform(OSPlatform.Linux))
            {
                ConfigFolder = "../ConfigApp";
                ConfigLogFolder = "../ConfigLog/nlog.config";
            }
            #endif
            var logger = NLogBuilder.ConfigureNLog(ConfigLogFolder).GetCurrentClassLogger();
            IConfiguration configuration = null;
            var builder = WebApplication.CreateBuilder(args);
            builder.WebHost.ConfigureAppConfiguration(
            (hostingContext, config) =>
            {
                var path = Path.Combine(Path.GetFullPath(ConfigFolder), "appsettings.json");
                config.AddJsonFile(path, optional: false, reloadOnChange: true);
                config.AddEnvironmentVariables();
                configuration = config.Build();

            });
            builder.WebHost.UseKestrel(o =>
            {
                o.ListenAnyIP(ConfigData.PortMqtt, l => l.UseMqtt());


            });
            // Add services to the container.
            builder.Services.AddControllers();
            builder.Services.AddControllersWithViews();
            builder.Services.AddCors(options =>
            {
                options.AddPolicy("AllowAllCors", builder =>
                {
                    builder.AllowAnyOrigin().AllowAnyMethod().AllowAnyHeader().WithExposedHeaders("content-disposition", "content-length", "content-type");
                });
            });
            builder.WebHost.ConfigureKestrel(serverOptions =>
            {
                serverOptions.Limits.MaxRequestBodySize = 2147483648; // 2GB
            });

            builder.Services.AddControllers();

            builder.Host.UseNLog();
            ConfigData.InitConfig(builder.Configuration);


            builder.Services.AddHttpClient().ConfigureHttpClientDefaults(http =>
            {
                http.ConfigurePrimaryHttpMessageHandler(() =>
                {
                    HttpClientHandler handler = new HttpClientHandler
                    {
                        ServerCertificateCustomValidationCallback = (message, cert, chain, errors) => true
                    };
                    return handler;
                });
            });
            Startup.InitServices(builder.Services);
            // Configure Database Connection

            builder.Services.AddEndpointsApiExplorer();
            builder.Services.AddSwaggerGen(c =>
            {
                c.SwaggerDoc("v1", new OpenApiInfo { Title = "My API", Version = "v1" });

            });

            var app = builder.Build();

            // Configure the HTTP request pipeline.

            // Enable developer exception page for detailed error info in Development mode
            app.UseDeveloperExceptionPage();

            // Enable Swagger only in Development environment
            app.UseSwagger();
            app.UseSwaggerUI(c =>
            {
                c.SwaggerEndpoint("/swagger/v1/swagger.json", "My API V1");
            });

            // Use a generic error handler for production environments
            app.UseExceptionHandler("/Home/Error");
            app.UseHsts();


            app.UseCors("AllowAllCors");
            app.UseHttpsRedirection();
            app.UseStaticFiles();

            app.UseRouting();
            app.UseEndpoints(endpoints =>
            {

                endpoints.MapControllers(); // For JSON API
                endpoints.MapDefaultControllerRoute(); // If using MVC
                //endpoints.MapMqtt("");
            });

            app.MapControllers();
            app.UseMqttServer(server =>
            {
                server.WithAttributeRouting(app.Services, allowUnmatchedRoutes: false);
            });

            app.MapConnectionHandler<MqttConnectionHandler>("/mqtt", o =>
            {
                o.WebSockets.SubProtocolSelector =
                    MQTTnet.AspNetCore.MqttSubProtocolSelector.SelectSubProtocol;
            });

            app.UseAuthorization();

            // Define the default route for MVC
            app.MapControllerRoute(
                name: "default",
                pattern: "{controller=Home}/{action=Index}/{id?}");

            app.Run();
        }

    }
}