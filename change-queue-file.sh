printf '<!DOCTYPE html>
<html lang="en">
<head>
    <meta http-equiv="Cache-Control" content="no-cache, no-store, must-revalidate">
    <meta http-equiv="Pragma" content="no-cache">
    <meta http-equiv="Expires" content="0">
    <meta charset="UTF-8">
    <title>Redirecting...</title>
    <script type="text/javascript">
        window.location.href = "%s";
    </script>
</head>
<body>
    <p>If you are not redirected, <a href="%s">click here</a>.</p>
</body>
</html>' $1 $1 > /config/www/queue.html