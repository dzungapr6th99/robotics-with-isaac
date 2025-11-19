using CommonLib;
using ConfigApp;
using Microsoft.AspNetCore.Hosting;
using Microsoft.OpenApi.Models;
using NLog.Web;
using RobotClient.Startups;
using RobotClient.Worker;
using System.Reflection.Metadata;

namespace RobotClient
{
    public class Program
    {
        public static void Main(string[] args)
        {
            string ConfigFolder = "ConfigApp";
            string ConfigLogFolder = "ConfigLog/nlog.config";
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
            // Add services to the container.
            builder.Services.AddControllers();
            builder.Services.AddControllersWithViews();
            builder.Services.AddCors(options =>
            {
                options.AddPolicy("AllowAllCors", builder =>
                {
                    builder.AllowAnyOrigin().AllowAnyMethod().AllowAnyHeader();
                });
            });
            builder.WebHost.ConfigureKestrel(serverOptions =>
            {
                serverOptions.Limits.MaxRequestBodySize = 2147483648; // 2GB
            });

            builder.Services.AddControllers();

            builder.Host.UseNLog();
            ConfigData.LoadConfig(builder.Configuration);


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
            StartupApp.InitServices(builder.Services);
            builder.Services.AddHostedService<RobotWorker>();
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
            });
            app.UseAuthorization();

            app.MapControllers();

            // Define the default route for MVC
            app.MapControllerRoute(
                name: "default",
                pattern: "{controller=Home}/{action=Index}/{id?}");

            app.Run();
        }
    }
}

